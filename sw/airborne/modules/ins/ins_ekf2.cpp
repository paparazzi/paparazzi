/*
 * Copyright (C) 2022-2026 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/ins/ins_ekf2.cpp
 *
 * INS based in the EKF2 of PX4
 *
 * The EKF2 and the PX4 libraries it needs are copied in sw/ext/px4_ekf2
 * (see sync_px4_ekf2.py there). The GNSS checks, which PX4 runs in its sensors
 * module, are done here before the GNSS data is given to the EKF.
 */

#include "modules/ins/ins_ekf2.h"
#include "modules/nav/waypoints.h"
#include "modules/core/abi.h"
#include "stabilization/stabilization_attitude.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "EKF/ekf.h"
#include "lib/gnss/gnss_checks.hpp"
#include "lib/world_magnetic_model/geo_mag_declination.h"
#include "math/pprz_isa.h"
#include "math/pprz_geodetic_wgs84.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"

using namespace estimator;

/** For SITL and NPS we need special includes */
#if defined SITL && USE_NPS
#include "nps_autopilot.h"
#include <stdio.h>
#endif

/* Options of the previous EKF2 version, replaced by the PX4 parameters (see conf/modules/ins_ekf2.xml) */
#if defined(INS_EKF2_FUSION_MODE) || defined(INS_EKF2_VDIST_SENSOR_TYPE) || defined(INS_EKF2_MAG_FUSION_TYPE)
#error "INS_EKF2_FUSION_MODE, VDIST_SENSOR_TYPE and MAG_FUSION_TYPE are replaced by INS_EKF2_GPS_CTRL, EV_CTRL, OF_CTRL, DRAG_CTRL, HGT_REF and MAG_TYPE"
#endif
#if defined(INS_EKF2_GPS_CHECK_MASK) || defined(INS_EKF2_GPS_REQ_EPH) || defined(INS_EKF2_GPS_REQ_EPV) || defined(INS_EKF2_GPS_REQ_SACC) || defined(INS_EKF2_GPS_REQ_NSATS) || defined(INS_EKF2_GPS_REQ_PDOP)
#error "INS_EKF2_GPS_CHECK_MASK and GPS_REQ_* are replaced by INS_EKF2_GNSS_CHECK and GNSS_REQ_*"
#endif
#if defined(INS_EKF2_FLOW_SENSOR_DELAY) || defined(INS_EKF2_MIN_FLOW_QUALITY) || defined(INS_EKF2_MAX_FLOW_RATE) || defined(INS_EKF2_FLOW_NOISE) || defined(INS_EKF2_FLOW_NOISE_QMIN) || defined(INS_EKF2_FLOW_INNOV_GATE) || defined(INS_EKF2_FLOW_POS_X)
#error "INS_EKF2_FLOW_* are replaced by INS_EKF2_OF_DELAY, OF_QMIN, OF_MAX_RATE, OF_N_MIN, OF_N_MAX, OF_GATE and OF_POS_*"
#endif
#if defined(INS_EKF2_SONAR_MIN_RANGE) || defined(INS_EKF2_SONAR_MAX_RANGE) || defined(INS_EKF2_RANGE_MAIN_AGL)
#error "INS_EKF2_SONAR_* and RANGE_MAIN_AGL are replaced by INS_EKF2_RNG_MIN_DIST, RNG_MAX_DIST and RNG_CTRL"
#endif

/** INS reference from flight plan, true by default */
#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
#endif

/*
 * EKF2 parameters
 *
 * All PX4 EKF2 parameters have the PX4 default and can be changed in the airframe as
 * INS_EKF2_<name> for EKF2_<name>, e.g. INS_EKF2_GPS_CTRL, INS_EKF2_HGT_REF, INS_EKF2_MAG_TYPE
 * (see generated/ekf2_params.h in sw/ext/px4_ekf2 and the PX4 parameter reference).
 * The GNSS checks are INS_EKF2_GNSS_<name> for GNSS_<name>.
 */

/** GNSS checks (INS_EKF2_GNSS_CHECK), bits of the PX4 GNSS_CHECK parameter */
#define GNSS_CHECK_NSATS   (1<<0)  ///< number of satellites (INS_EKF2_GNSS_REQ_NSATS)
#define GNSS_CHECK_PDOP    (1<<1)  ///< position dilution of precision (INS_EKF2_GNSS_REQ_PDOP)
#define GNSS_CHECK_EPH     (1<<2)  ///< horizontal position accuracy (INS_EKF2_GNSS_REQ_EPH)
#define GNSS_CHECK_EPV     (1<<3)  ///< vertical position accuracy (INS_EKF2_GNSS_REQ_EPV)
#define GNSS_CHECK_SACC    (1<<4)  ///< speed accuracy (INS_EKF2_GNSS_REQ_SACC)
#define GNSS_CHECK_HDRIFT  (1<<5)  ///< horizontal position drift at rest on the ground (INS_EKF2_GNSS_REQ_HDRIFT)
#define GNSS_CHECK_VDRIFT  (1<<6)  ///< vertical position drift at rest on the ground (INS_EKF2_GNSS_REQ_VDRIFT)
#define GNSS_CHECK_HSPEED  (1<<7)  ///< horizontal speed at rest on the ground (INS_EKF2_GNSS_REQ_HDRIFT)
#define GNSS_CHECK_VSPEED  (1<<8)  ///< vertical speed at rest on the ground (INS_EKF2_GNSS_REQ_VDRIFT)
#define GNSS_CHECK_SPOOFED (1<<9)  ///< receiver reports spoofing
#define GNSS_CHECK_FIX     (1<<10) ///< fix type (INS_EKF2_GNSS_REQ_FIX)
#define GNSS_CHECK_JAMMED  (1<<11) ///< receiver reports jamming

/** GNSS checks used, PX4 enables all except jamming (2047) */
#ifndef INS_EKF2_GNSS_CHECK
#define INS_EKF2_GNSS_CHECK (GNSS_CHECK_NSATS | GNSS_CHECK_EPH | GNSS_CHECK_SACC | GNSS_CHECK_FIX)
#endif
PRINT_CONFIG_VAR(INS_EKF2_GNSS_CHECK)

/** Values of INS_EKF2_GPS_CTRL (bitmask) */
#define EKF2_GPS_CTRL_HPOS 1  ///< horizontal position
#define EKF2_GPS_CTRL_VPOS 2  ///< vertical position
#define EKF2_GPS_CTRL_VEL  4  ///< 3D velocity
#define EKF2_GPS_CTRL_YAW  8  ///< dual antenna heading

/** Values of INS_EKF2_EV_CTRL (bitmask) */
#define EKF2_EV_CTRL_HPOS 1  ///< horizontal position
#define EKF2_EV_CTRL_VPOS 2  ///< vertical position
#define EKF2_EV_CTRL_VEL  4  ///< 3D velocity
#define EKF2_EV_CTRL_YAW  8  ///< yaw

/** Values of INS_EKF2_HGT_REF */
#define EKF2_HGT_REF_BARO  0
#define EKF2_HGT_REF_GNSS  1
#define EKF2_HGT_REF_RANGE 2
#define EKF2_HGT_REF_EV    3

/** Values of INS_EKF2_MAG_TYPE */
#define EKF2_MAG_TYPE_AUTO    0  ///< heading on the ground, 3D in the air
#define EKF2_MAG_TYPE_HEADING 1  ///< heading only
#define EKF2_MAG_TYPE_NONE    5  ///< not used
#define EKF2_MAG_TYPE_INIT    6  ///< only to initialize the heading

/** Indoor configuration with only external vision (e.g. Optitrack): position, velocity, yaw and height
 *  from external vision, no GNSS, magnetometer and barometer, and the external vision noise and delay
 *  of the previous EKF2 version. Each value can still be changed in the airframe. */
#if INS_EKF2_OPTITRACK
#ifndef INS_EKF2_GPS_CTRL
#define INS_EKF2_GPS_CTRL 0
#endif
#ifndef INS_EKF2_EV_CTRL
#define INS_EKF2_EV_CTRL (EKF2_EV_CTRL_HPOS | EKF2_EV_CTRL_VPOS | EKF2_EV_CTRL_VEL | EKF2_EV_CTRL_YAW)
#endif
#ifndef INS_EKF2_HGT_REF
#define INS_EKF2_HGT_REF EKF2_HGT_REF_EV
#endif
#ifndef INS_EKF2_MAG_TYPE
#define INS_EKF2_MAG_TYPE EKF2_MAG_TYPE_NONE
#endif
#ifndef INS_EKF2_BARO_CTRL
#define INS_EKF2_BARO_CTRL 0
#endif
#ifndef INS_EKF2_EVP_NOISE
#define INS_EKF2_EVP_NOISE 0.02f
#endif
#ifndef INS_EKF2_EVA_NOISE
#define INS_EKF2_EVA_NOISE 0.05f
#endif
#ifndef INS_EKF2_EV_DELAY
#define INS_EKF2_EV_DELAY 175.f
#endif
#endif

/** Barometer as height reference (PX4 uses GNSS), as the previous version: the
 *  heights don't depend on the ground altitude of the flight plan */
#ifndef INS_EKF2_HGT_REF
#define INS_EKF2_HGT_REF EKF2_HGT_REF_BARO
#endif

/* Vehicle specific defaults, as PX4 (rc.fw_defaults and rc.mc_defaults) */
#if defined(FIXEDWING_FIRMWARE)
#ifndef INS_EKF2_ARSP_THR
#define INS_EKF2_ARSP_THR 8.f     ///< airspeed fusion above 8 m/s
#endif
#ifndef INS_EKF2_FUSE_BETA
#define INS_EKF2_FUSE_BETA 1      ///< sideslip fusion
#endif
#ifndef INS_EKF2_MAG_ACCLIM
#define INS_EKF2_MAG_ACCLIM 0.f
#endif
#ifndef INS_EKF2_RNG_QLTY_T
#define INS_EKF2_RNG_QLTY_T 3.f
#endif
#else
#ifndef INS_EKF2_RNG_FOG
#define INS_EKF2_RNG_FOG 1.f
#endif
#endif

/* All parameters with the PX4 defaults, the values above can be used in the airframe */
#include "generated/ekf2_params.h"

PRINT_CONFIG_VAR(INS_EKF2_GPS_CTRL)
PRINT_CONFIG_VAR(INS_EKF2_EV_CTRL)
PRINT_CONFIG_VAR(INS_EKF2_HGT_REF)
PRINT_CONFIG_VAR(INS_EKF2_MAG_TYPE)

/** GNSS measurement delay (ms), as the PX4 SENS_GNSS0_DELAY parameter */
#ifndef INS_EKF2_GPS_DELAY
#define INS_EKF2_GPS_DELAY 110
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_DELAY)

/** Minimum distance the range finder can measure (m) */
#ifndef INS_EKF2_RNG_MIN_DIST
#define INS_EKF2_RNG_MIN_DIST 0.001f
#endif
PRINT_CONFIG_VAR(INS_EKF2_RNG_MIN_DIST)

/** Maximum distance the range finder can measure (m) */
#ifndef INS_EKF2_RNG_MAX_DIST
#define INS_EKF2_RNG_MAX_DIST 4.f
#endif
PRINT_CONFIG_VAR(INS_EKF2_RNG_MAX_DIST)

/** Maximum flow rate the optical flow sensor can measure (rad/s) */
#ifndef INS_EKF2_OF_MAX_RATE
#define INS_EKF2_OF_MAX_RATE 200.f
#endif
PRINT_CONFIG_VAR(INS_EKF2_OF_MAX_RATE)

/** default airspeed sensor to use in INS (equivalent airspeed, only fused above INS_EKF2_ARSP_THR):
 *  any sensor, or the airspeed computed by air_data from a differential pressure sensor.
 *  The airspeed estimated from the wind (wind_estimation_quadrotor) is never used. */
#ifndef INS_EKF2_AIRSPEED_ID
#define INS_EKF2_AIRSPEED_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_AIRSPEED_ID)

/** default barometer to use in INS */
#ifndef INS_EKF2_BARO_ID
#if USE_BARO_BOARD
#define INS_EKF2_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_EKF2_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_EKF2_BARO_ID)

/** default temperature sensor to use in INS */
#ifndef INS_EKF2_TEMPERATURE_ID
#define INS_EKF2_TEMPERATURE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_TEMPERATURE_ID)

/** default AGL sensor to use in INS */
#ifndef INS_EKF2_AGL_ID
#define INS_EKF2_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_AGL_ID)

/* default Gyro to use in INS */
#ifndef INS_EKF2_GYRO_ID
#define INS_EKF2_GYRO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_GYRO_ID)

/* default Accelerometer to use in INS */
#ifndef INS_EKF2_ACCEL_ID
#define INS_EKF2_ACCEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_ACCEL_ID)

/* default Magnetometer to use in INS */
#ifndef INS_EKF2_MAG_ID
#define INS_EKF2_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_MAG_ID)

/* default GPS to use in INS */
#ifndef INS_EKF2_GPS_ID
#define INS_EKF2_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_ID)

/* default RELPOS to use for heading in INS */
#ifndef INS_EKF2_RELPOS_ID
#define INS_EKF2_RELPOS_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_RELPOS_ID)

/* default Optical Flow to use in INS */
#ifndef INS_EKF2_OF_ID
#define INS_EKF2_OF_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_OF_ID)

/* GPS antenna position from the CoG in meters, as the PX4 SENS_GNSS0_OFF* parameters */
#ifndef INS_EKF2_GPS_POS_X
#define INS_EKF2_GPS_POS_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_X)

#ifndef INS_EKF2_GPS_POS_Y
#define INS_EKF2_GPS_POS_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_Y)

#ifndef INS_EKF2_GPS_POS_Z
#define INS_EKF2_GPS_POS_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_Z)

/* GNSS heading measurement noise when the receiver gives no accuracy (rad) */
#ifndef INS_EKF2_GPS_YAW_NOISE
#define INS_EKF2_GPS_YAW_NOISE 0.1f
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_YAW_NOISE)

/* Maximum difference between the measured and the configured (INS_EKF2_GPS_ANTENNA_DISTANCE) dual antenna distance (m), the heading is not used otherwise */
#ifndef INS_EKF2_GPS_ANTENNA_DISTANCE_ERR
#define INS_EKF2_GPS_ANTENNA_DISTANCE_ERR 0.2
#endif

#ifdef INS_EXT_VISION_ROTATION
struct FloatQuat ins_ext_vision_rot;
#endif

/* All registered ABI events */
static abi_event baro_ev;
static abi_event temperature_ev;
static abi_event agl_ev;
static abi_event gyro_int_ev;
static abi_event accel_int_ev;
static abi_event mag_ev;
static abi_event gps_ev;
static abi_event relpos_ev;
static abi_event optical_flow_ev;
static abi_event airspeed_ev;
static abi_event reset_ev;

/* All ABI callbacks */
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);
static void temperature_cb(uint8_t sender_id, float temp);
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);
static void gyro_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatRates *delta_gyro, uint16_t dt);
static void accel_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatVect3 *delta_accel, uint16_t dt);
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void relpos_cb(uint8_t sender_id, uint32_t stamp, struct RelPosNED *relpos);
static void optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x, int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);
static void airspeed_cb(uint8_t sender_id, float eas);
static void reset_cb(uint8_t sender_id, uint8_t flag);

/* Static local functions */
static void ins_ekf2_publish_attitude(uint64_t time_us);

/* Static local variables */
static Ekf ekf;                                   ///< EKF class itself
static parameters *ekf_params;                    ///< The EKF parameters
static GnssChecks gnss_checks;                    ///< GNSS checks before the GNSS data is used
struct ekf2_t ekf2;                               ///< Local EKF2 status structure
static struct extVisionSample sample_ev;          ///< External vision sample

/**
 * Convert a 32 bit timestamp (us) of an ABI message to the 64 bit time of the EKF,
 * so the EKF time keeps increasing when the 32 bit time wraps (after 71 minutes)
 */
static uint64_t ekf2_time_us(uint32_t stamp)
{
  static uint32_t last_now = 0;
  static uint64_t wraps = 0;
  uint32_t now = get_sys_time_usec();
  if (now < last_now) {
    wraps += (1ULL << 32);
  }
  last_now = now;

  // A stamp more than half the range later than now was taken before the last wrap
  uint64_t time_us = wraps + stamp;
  if (stamp > now && (stamp - now) > (1UL << 31) && time_us >= (1ULL << 32)) {
    time_us -= (1ULL << 32);
  }
  return time_us;
}

/** Time of a measurement that was taken delay_ms before it arrived */
static uint64_t ekf2_delayed_time_us(uint32_t stamp, uint32_t delay_ms)
{
  uint64_t time_us = ekf2_time_us(stamp);
  return (time_us > delay_ms * 1000ULL) ? time_us - delay_ms * 1000ULL : 0;
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i pos, speed, accel;

  // Get it from the EKF
  const Vector3f pos_f{ekf.getPosition()};
  const Vector3f speed_f{ekf.getVelocity()};

  // Convert to integer
  pos.x = POS_BFP_OF_REAL(pos_f(0));
  pos.y = POS_BFP_OF_REAL(pos_f(1));
  pos.z = POS_BFP_OF_REAL(pos_f(2));
  speed.x = SPEED_BFP_OF_REAL(speed_f(0));
  speed.y = SPEED_BFP_OF_REAL(speed_f(1));
  speed.z = SPEED_BFP_OF_REAL(speed_f(2));
  accel.x = ACCEL_BFP_OF_REAL(ekf2.accel_ned.x);
  accel.y = ACCEL_BFP_OF_REAL(ekf2.accel_ned.y);
  accel.z = ACCEL_BFP_OF_REAL(ekf2.accel_ned.z);

  // Send the message
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &pos.x, &pos.y, &pos.z,
                    &speed.x, &speed.y, &speed.z,
                    &accel.x, &accel.y, &accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  float baro_z = 0.0f;
  int32_t pos_z, speed_z, accel_z;

  // Get it from the EKF
  const Vector3f pos_f{ekf.getPosition()};
  const Vector3f speed_f{ekf.getVelocity()};

  // Convert to integer
  pos_z = POS_BFP_OF_REAL(pos_f(2));
  speed_z = SPEED_BFP_OF_REAL(speed_f(2));
  accel_z = ACCEL_BFP_OF_REAL(ekf2.accel_ned.z);

  // Send the message
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &baro_z, &pos_z, &speed_z, &accel_z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  float qfe = 101325.0; //TODO: this is qnh not qfe?
  if (ekf2.ltp_stamp > 0)
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ekf2.ltp_def.ecef.x, &ekf2.ltp_def.ecef.y, &ekf2.ltp_def.ecef.z,
                          &ekf2.ltp_def.lla.lat, &ekf2.ltp_def.lla.lon, &ekf2.ltp_def.lla.alt,
                          &ekf2.ltp_def.hmsl, &qfe);
}

/** Innovation test ratio, 0 when not available */
static float test_ratio(float ratio)
{
  return PX4_ISFINITE(ratio) ? ratio : 0.f;
}

static void send_ins_ekf2(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t control_mode = ekf.control_status().value; // lower 32 bits of the status flags
  uint16_t filter_fault_status = ekf.fault_status().value;
  uint16_t gps_check_status = gnss_checks.getStrictFailFlags();
  uint16_t soln_status = ekf.get_ekf_soln_status();

  // Innovation test ratios, a bit is set in innov_test_status when the test fails (ratio > 1)
  float mag = test_ratio(ekf.getHeadingInnovationTestRatio());
  float vel = test_ratio(math::max(test_ratio(ekf.getHorizontalVelocityInnovationTestRatio()),
                                   test_ratio(ekf.getVerticalVelocityInnovationTestRatio())));
  float pos = test_ratio(ekf.getHorizontalPositionInnovationTestRatio());
  float hgt = test_ratio(ekf.getVerticalPositionInnovationTestRatio());
  float tas = test_ratio(ekf.getAirspeedInnovationTestRatio());
  float hagl = test_ratio(ekf.getHeightAboveGroundInnovationTestRatio());
  float beta = test_ratio(ekf.getSyntheticSideslipInnovationTestRatio());
  float flow = test_ratio(math::max(test_ratio(ekf.aid_src_optical_flow().test_ratio[0]),
                                    test_ratio(ekf.aid_src_optical_flow().test_ratio[1])));
  const float ratios[] = {mag, vel, pos, hgt, tas, hagl, flow, beta};
  uint16_t innov_test_status = 0;
  for (uint8_t i = 0; i < sizeof(ratios) / sizeof(ratios[0]); i++) {
    if (ratios[i] > 1.f) {
      innov_test_status |= (1 << i);
    }
  }

  float mag_decl = 0.0f;
  ekf.get_mag_decl_deg(mag_decl);

  uint8_t terrain_valid = ekf.isTerrainEstimateValid() ? 1 : 0;
  uint8_t dead_reckoning = ekf.control_status_flags().inertial_dead_reckoning ? 1 : 0;

  pprz_msg_send_INS_EKF2(trans, dev, AC_ID,
                         &control_mode, &filter_fault_status, &gps_check_status, &soln_status,
                         &innov_test_status, &mag, &vel, &pos, &hgt, &tas, &hagl, &flow, &beta,
                         &mag_decl, &terrain_valid, &dead_reckoning);
}

static void send_ins_ekf2_ext(struct transport_tx *trans, struct link_device *dev)
{
  // GNSS drift on the ground, as computed by the GNSS checks
  float gps_drift[3] = {
    gnss_checks.horizontal_position_drift_rate_m_s(),
    gnss_checks.vertical_position_drift_rate_m_s(),
    gnss_checks.filtered_horizontal_velocity_m_s()
  };
  uint8_t gps_blocked = !PX4_ISFINITE(gps_drift[0]);

  // Upper 32 bits of the control status (the lower ones are in INS_EKF2)
  uint32_t control_mode_upper = (uint32_t)(ekf.control_status().value >> 32);

  // Estimated accuracies
  float pos_acc_h, pos_acc_v, vel_acc_h, vel_acc_v;
  ekf.get_ekf_gpos_accuracy(&pos_acc_h, &pos_acc_v);
  ekf.get_ekf_vel_accuracy(&vel_acc_h, &vel_acc_v);

  // Height above ground and height sensor biases
  float hagl = ekf.isTerrainEstimateValid() ? ekf.getHagl() : NAN;
  float baro_bias = ekf.getBaroBiasEstimatorStatus().bias;
  float gnss_hgt_bias = ekf.getGpsHgtBiasEstimatorStatus().bias;

  // EKF-GSF emergency yaw estimate (independent of the magnetometer)
  float yaw_gsf = NAN, yaw_gsf_var = NAN;
  float gsf_yaw[N_MODELS_EKFGSF], gsf_innov_vn[N_MODELS_EKFGSF], gsf_innov_ve[N_MODELS_EKFGSF], gsf_weight[N_MODELS_EKFGSF];
  if (!ekf.getDataEKFGSF(&yaw_gsf, &yaw_gsf_var, gsf_yaw, gsf_innov_vn, gsf_innov_ve, gsf_weight)) {
    yaw_gsf = NAN;
    yaw_gsf_var = NAN;
  }
  float yaw_gsf_std = (PX4_ISFINITE(yaw_gsf_var) && yaw_gsf_var >= 0.f) ? sqrtf(yaw_gsf_var) : NAN;

  // Magnetometer checks against the world magnetic model
  float mag_inc, mag_inc_ref, mag_strength, mag_strength_ref;
  ekf.get_mag_checks(mag_inc, mag_inc_ref, mag_strength, mag_strength_ref);

  pprz_msg_send_INS_EKF2_EXT(trans, dev, AC_ID,
                             &gps_drift[0], &gps_drift[1], &gps_drift[2], &gps_blocked,
                             &control_mode_upper,
                             &pos_acc_h, &pos_acc_v, &vel_acc_h, &vel_acc_v,
                             &hagl, &baro_bias, &gnss_hgt_bias,
                             &yaw_gsf, &yaw_gsf_std,
                             &mag_inc, &mag_inc_ref, &mag_strength, &mag_strength_ref);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t ahrs_ekf2_id = AHRS_COMP_ID_EKF2;
  const auto &flags = ekf.control_status_flags();
  uint32_t filter_fault_status = ekf.fault_status().value;
  uint16_t filter_fault_status_16 = filter_fault_status; //FIXME
  uint8_t mde = 0;

  // Check the alignment and if GPS is fused
  if (flags.tilt_align && flags.yaw_align && (flags.gnss_pos || flags.ev_pos)) {
    mde = 3;
  } else if (flags.tilt_align && flags.yaw_align) {
    mde = 4;
  } else {
    mde = 2;
  }

  // Check if there is a covariance error
  if (filter_fault_status) {
    mde = 6;
  }

  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_ekf2_id, &mde, &filter_fault_status_16);
}

static void send_wind_info_ret(struct transport_tx *trans, struct link_device *dev)
{
  Vector2f wind = ekf.getWindVelocity();
  const Vector3f vel = ekf.getVelocity();
  // True airspeed from the velocity and the wind estimate
  float tas = Vector3f(vel(0) - wind(0), vel(1) - wind(1), vel(2)).norm();
  uint8_t flags = 0x5;
  float f_zero = 0;

  pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID, &flags, &wind(1), &wind(0), &f_zero, &tas);
}

static void send_ahrs_bias(struct transport_tx *trans, struct link_device *dev)
{
  Vector3f accel_bias = ekf.getAccelBias();
  Vector3f gyro_bias = ekf.getGyroBias();
  Vector3f mag_bias = ekf.getMagBias();

  pprz_msg_send_AHRS_BIAS(trans, dev, AC_ID, &accel_bias(0), &accel_bias(1), &accel_bias(2),
                          &gyro_bias(0), &gyro_bias(1), &gyro_bias(2), &mag_bias(0), &mag_bias(1), &mag_bias(2));
}

static void send_ahrs_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat ltp_to_body_quat;
  const Quatf att_q{ekf.getQuaternion()};
  ltp_to_body_quat.qi = QUAT1_BFP_OF_REAL(att_q(0));
  ltp_to_body_quat.qx = QUAT1_BFP_OF_REAL(att_q(1));
  ltp_to_body_quat.qy = QUAT1_BFP_OF_REAL(att_q(2));
  ltp_to_body_quat.qz = QUAT1_BFP_OF_REAL(att_q(3));
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  float foo = 0.f;
  uint8_t ahrs_id = 1; // generic
  pprz_msg_send_AHRS_QUAT_INT(trans, dev, AC_ID,
                              &foo,
                              &ltp_to_body_quat.qi,
                              &ltp_to_body_quat.qx,
                              &ltp_to_body_quat.qy,
                              &ltp_to_body_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz),
                              &ahrs_id);
}


static void send_external_pose_down(struct transport_tx *trans, struct link_device *dev)
{
  if (sample_ev.time_us == 0) {
    return;
  }
  float sample_temp_ev[11];
  sample_temp_ev[0]  = (float) sample_ev.time_us;
  sample_temp_ev[1]  = sample_ev.pos(0) ;
  sample_temp_ev[2]  = sample_ev.pos(1) ;
  sample_temp_ev[3]  = sample_ev.pos(2) ;
  sample_temp_ev[4]  = sample_ev.vel(0) ;
  sample_temp_ev[5]  = sample_ev.vel(1) ;
  sample_temp_ev[6]  = sample_ev.vel(2) ;
  sample_temp_ev[7]  = sample_ev.quat(0);
  sample_temp_ev[8]  = sample_ev.quat(1);
  sample_temp_ev[9]  = sample_ev.quat(2);
  sample_temp_ev[10] = sample_ev.quat(3);
  pprz_msg_send_EXTERNAL_POSE_DOWN(trans, dev, AC_ID,
                                   &sample_temp_ev[0],
                                   &sample_temp_ev[1],
                                   &sample_temp_ev[2],
                                   &sample_temp_ev[3],
                                   &sample_temp_ev[4],
                                   &sample_temp_ev[5],
                                   &sample_temp_ev[6],
                                   &sample_temp_ev[7],
                                   &sample_temp_ev[8],
                                   &sample_temp_ev[9],
                                   &sample_temp_ev[10]);
}
#endif

/* Initialize the EKF */
void ins_ekf2_init(void)
{
  /* Get the ekf parameters, all with the PX4 defaults or the airframe values */
  ekf_params = ekf.getParamHandle();
  ins_ekf2_set_px4_params(ekf_params);

  /* The observation buffers must also hold the GNSS delay, which is not an EKF parameter */
  ekf_params->ekf2_delay_max = math::max((float)INS_EKF2_DELAY_MAX, (float)INS_EKF2_GPS_DELAY);

  /* Output predictor, as in the PX4 EKF2 module */
  ekf.output_predictor().set_imu_offset(ekf_params->imu_pos_body);
  ekf.output_predictor().set_pos_correction_tc(INS_EKF2_TAU_POS);
  ekf.output_predictor().set_vel_correction_tc(INS_EKF2_TAU_VEL);
  ekf.set_gnss_yaw_reset_wait(INS_EKF2_GPS_YAW_RST * 1000000);

  /* Enable all sensors, which ones are used follows from the parameters above */
  FusionControl *fc = ekf.getFusionControlHandle();
  fc->gps.enabled = true;
  fc->of.enabled = true;
  fc->ev.enabled = true;
  fc->baro.enabled = true;
  fc->rng.enabled = true;
  fc->mag.enabled = true;
  fc->aspd.enabled = true;
  ekf.updateParameters();

  /* GNSS checks before the GNSS data is used, as the PX4 GNSS_* parameters */
  GnssChecks::Params checks_params{};
  checks_params.check_mask = INS_EKF2_GNSS_CHECK;
  checks_params.req_nsats = INS_EKF2_GNSS_REQ_NSATS;
  checks_params.req_pdop = INS_EKF2_GNSS_REQ_PDOP;
  checks_params.req_eph = INS_EKF2_GNSS_REQ_EPH;
  checks_params.req_epv = INS_EKF2_GNSS_REQ_EPV;
  checks_params.req_sacc = INS_EKF2_GNSS_REQ_SACC;
  checks_params.req_hdrift = INS_EKF2_GNSS_REQ_HDRIFT;
  checks_params.req_vdrift = INS_EKF2_GNSS_REQ_VDRIFT;
  checks_params.req_fix = INS_EKF2_GNSS_REQ_FIX;
  checks_params.min_health_time_us = INS_EKF2_GNSS_REQ_TIME * 1000000;
  gnss_checks.setParams(checks_params);

  /* Initialize struct */
  ekf2.ltp_stamp = 0;
  ekf2.flow_stamp = 0;
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = false;
  ekf2.quat_reset_counter = 0;
  ekf2.temp = 20.0f; // Default temperature of 20 degrees celcius
  ekf2.in_air = false;
  ekf2.armed = false;
  ekf2.fixed_wing = false;
  ekf2.air_density = PPRZ_ISA_AIR_DENSITY;
  /* Field strength of the world magnetic model for the normalized magnetometer */
  ekf2.mag_scale = 0.4f;
#if USE_INS_NAV_INIT
  float strength = get_mag_strength_gauss(NAV_LAT0 * 1e-7f, NAV_LON0 * 1e-7f);
  if (PX4_ISFINITE(strength) && strength > 0.1f && strength < 1.f) {
    ekf2.mag_scale = strength;
  }
#endif
  ekf2.system_flags_time = 0;
  FLOAT_VECT3_ZERO(ekf2.accel_ned);

  /* Initialize the range sensor limits */
  ekf.set_rangefinder_limits(INS_EKF2_RNG_MIN_DIST, INS_EKF2_RNG_MAX_DIST);

  /* Initialize the flow sensor limits */
  ekf.set_optical_flow_limits(INS_EKF2_OF_MAX_RATE, INS_EKF2_RNG_MIN_DIST, INS_EKF2_RNG_MAX_DIST);

  // Don't send external vision data by default
  sample_ev.time_us = 0;

  /* Initialize the origin from flight plan */
#if USE_INS_NAV_INIT
  if (ekf.setEkfGlobalOrigin(NAV_LAT0 * 1e-7, NAV_LON0 * 1e-7, (NAV_ALT0) * 1e-3)) { // EKF2 works HMSL
    struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
    llh_nav0.lat = NAV_LAT0;
    llh_nav0.lon = NAV_LON0;
    /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
    llh_nav0.alt = NAV_ALT0 + NAV_MSL0; // in millimeters above WGS84 reference ellipsoid

    ltp_def_from_lla_i(&ekf2.ltp_def, &llh_nav0);
    ekf2.ltp_def.hmsl = NAV_ALT0;
    stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);

    /* update local ENU coordinates of global waypoints */
    waypoints_localize_all();

    ekf2.ltp_stamp = 1;
  }
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_EKF2, send_ins_ekf2);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_EKF2_EXT, send_ins_ekf2_ext);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_INFO_RET, send_wind_info_ret);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_BIAS, send_ahrs_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_QUAT_INT, send_ahrs_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EXTERNAL_POSE_DOWN, send_external_pose_down);
#endif

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgBARO_ABS(INS_EKF2_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgTEMPERATURE(INS_EKF2_TEMPERATURE_ID, &temperature_ev, temperature_cb);
  AbiBindMsgAGL(INS_EKF2_AGL_ID, &agl_ev, agl_cb);
  AbiBindMsgIMU_GYRO_INT(INS_EKF2_GYRO_ID, &gyro_int_ev, gyro_int_cb);
  AbiBindMsgIMU_ACCEL_INT(INS_EKF2_ACCEL_ID, &accel_int_ev, accel_int_cb);
  AbiBindMsgIMU_MAG(INS_EKF2_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgGPS(INS_EKF2_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgRELPOS(INS_EKF2_RELPOS_ID, &relpos_ev, relpos_cb);
  AbiBindMsgOPTICAL_FLOW(INS_EKF2_OF_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgAIRSPEED(INS_EKF2_AIRSPEED_ID, &airspeed_ev, airspeed_cb);
  AbiBindMsgINS_RESET(ABI_BROADCAST, &reset_ev, reset_cb);
}

static void reset_ref(void)
{
#if USE_GPS
  if (GpsFixValid()) {
    struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
    if (ekf.setEkfGlobalOrigin(lla_pos.lat * 1e-7, lla_pos.lon * 1e-7, gps.hmsl * 1e-3)) {
      ltp_def_from_lla_i(&ekf2.ltp_def, &lla_pos);
      ekf2.ltp_def.hmsl = gps.hmsl;
      stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);
    }
  }
#endif
}

static void reset_vertical_ref(void)
{
#if USE_GPS
  if (GpsFixValid()) {
    struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
    struct LlaCoor_i lla = {
      .lat = stateGetLlaOrigin_i().lat,
      .lon = stateGetLlaOrigin_i().lon,
      .alt = lla_pos.alt
    };
    if (ekf.setEkfGlobalOrigin(lla.lat * 1e-7, lla.lon * 1e-7, gps.hmsl * 1e-3)) {
      ltp_def_from_lla_i(&ekf2.ltp_def, &lla);
      ekf2.ltp_def.hmsl = gps.hmsl;
      stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);
    }
  }
#endif
}

static void reset_cb(uint8_t sender_id UNUSED, uint8_t flag)
{
  switch (flag) {
    case INS_RESET_REF:
      reset_ref();
      break;
    case INS_RESET_VERTICAL_REF:
      reset_vertical_ref();
      break;
    default:
      // unsupported cases
      break;
  }
}

/** Give the arming and in-air state to the EKF when it changes, and every second
 *  (the EKF ignores them until it is initialised by the first IMU data) */
static void ins_ekf2_update_system_flags(void)
{
  bool in_air = autopilot_in_flight();
  bool armed = autopilot_get_motors_on();
  bool fixed_wing = ins_ekf2_fixed_wing_flight();
  uint64_t now = ekf2_time_us(get_sys_time_usec());
  if (in_air == ekf2.in_air && armed == ekf2.armed && fixed_wing == ekf2.fixed_wing
      && now < ekf2.system_flags_time + 1000000) {
    return;
  }

  systemFlagUpdate flags{};
  flags.time_us = now;
  flags.armed = armed;
  flags.in_air = in_air;
  flags.is_fixed_wing = fixed_wing;
  ekf.setSystemFlagData(flags);

  ekf2.in_air = in_air;
  ekf2.armed = armed;
  ekf2.fixed_wing = fixed_wing;
  ekf2.system_flags_time = now;
}

/** Flying as a fixed wing (airspeed and sideslip fusion, no ground effect, airspeed for the yaw estimator).
 *  True for the fixedwing firmware. Hybrids flown with the rotorcraft firmware (e.g. rotwing_state)
 *  override this weak function with their forward flight state. */
bool __attribute__((weak)) ins_ekf2_fixed_wing_flight(void)
{
#if defined(FIXEDWING_FIRMWARE)
  return true;
#else
  return false;
#endif
}

/** Airspeed (equivalent airspeed) to the EKF as true airspeed */
static void airspeed_cb(uint8_t sender_id, float eas)
{
  if (sender_id == AIRSPEED_WE_QUAD_ID || !PX4_ISFINITE(eas) || eas < 0.f || ekf2.air_density <= 0.f) {
    return;
  }
  airspeedSample sample{};
  sample.time_us = ekf2_time_us(get_sys_time_usec());
  sample.eas2tas = sqrtf(PPRZ_ISA_AIR_DENSITY / ekf2.air_density);
  sample.true_airspeed = eas * sample.eas2tas;
  ekf.setAirspeedData(sample);
}

/* Update the INS state */
void ins_ekf2_update(void)
{
  ins_ekf2_update_system_flags();

  /* Update the EKF */
  if (ekf2.got_imu_data) {
    // Update the EKF but ignore the response and also copy the faster intermediate filter
    ekf.update();

    /* Get the accelerations in NED frame (average since the last update) */
    const Vector3f vel_deriv_f{ekf.getVelocityDerivative()};
    ekf.resetVelocityDerivativeAccumulation();

    // Only publish position after successful alignment
    if (ekf.control_status_flags().tilt_align) {
      /* Get the position */
      const Vector3f pos_f{ekf.getPosition()};
      struct NedCoor_f pos;
      pos.x = pos_f(0);
      pos.y = pos_f(1);
      pos.z = pos_f(2);

      // Publish to the state
      stateSetPositionNed_f(MODULE_INS_EKF2_ID, &pos);

      /* Get the velocity in NED frame */
      const Vector3f vel_f{ekf.getVelocity()};
      struct NedCoor_f speed;
      speed.x = vel_f(0);
      speed.y = vel_f(1);
      speed.z = vel_f(2);

      // Publish to state
      stateSetSpeedNed_f(MODULE_INS_EKF2_ID, &speed);

      ekf2.accel_ned.x = vel_deriv_f(0);
      ekf2.accel_ned.y = vel_deriv_f(1);
      ekf2.accel_ned.z = vel_deriv_f(2);

      // Publish to state
      struct NedCoor_f accel;
      accel.x = ekf2.accel_ned.x;
      accel.y = ekf2.accel_ned.y;
      accel.z = ekf2.accel_ned.z;
      stateSetAccelNed_f(MODULE_INS_EKF2_ID, &accel);

      /* Get local origin */
      // Position of local NED origin in GPS / WGS84 frame
      double ekf_origin_lat, ekf_origin_lon;
      float ref_alt;
      struct LlaCoor_i lla_ref;
      uint64_t origin_time;

      // Only update the origin when the state estimator has updated the origin
      if (ekf.global_origin_valid()) {
        ekf.getEkfGlobalOrigin(origin_time, ekf_origin_lat, ekf_origin_lon, ref_alt);
      } else {
        origin_time = 0;
      }
      if (origin_time > ekf2.ltp_stamp) {
#if INS_EKF2_FORCE_NAV_REF && USE_INS_NAV_INIT
        if (ekf.setEkfGlobalOrigin(NAV_LAT0 * 1e-7, NAV_LON0 * 1e-7, (NAV_ALT0) * 1e-3)) {
          struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
          llh_nav0.lat = NAV_LAT0;
          llh_nav0.lon = NAV_LON0;
          llh_nav0.alt = NAV_ALT0 + NAV_MSL0; // in millimeters above WGS84 reference ellipsoid

          ltp_def_from_lla_i(&ekf2.ltp_def, &llh_nav0);
          ekf2.ltp_def.hmsl = NAV_ALT0;
          stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);
        }
#else
        lla_ref.lat = ekf_origin_lat * 1e7; // WGS-84 lat
        lla_ref.lon = ekf_origin_lon * 1e7; // WGS-84 lon
        lla_ref.alt = ref_alt * 1e3 + wgs84_ellipsoid_to_geoid_i(lla_ref.lat, lla_ref.lon); // in millimeters above WGS84 reference ellipsoid (ref_alt is in HMSL)
        ltp_def_from_lla_i(&ekf2.ltp_def, &lla_ref);
        ekf2.ltp_def.hmsl = ref_alt * 1e3;
        stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);
#endif
        /* update local ENU coordinates of global waypoints */
        waypoints_localize_all();

        ekf2.ltp_stamp = origin_time;
      }
    }
  }

#if defined SITL && USE_NPS
  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }
#endif

  ekf2.got_imu_data = false;
}

void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf)
{
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  sample_ev.time_us = ekf2_time_us(get_sys_time_usec()); //FIXME
  sample_ev.pos(0) = DL_EXTERNAL_POSE_enu_y(buf);
  sample_ev.pos(1) = DL_EXTERNAL_POSE_enu_x(buf);
  sample_ev.pos(2) = -DL_EXTERNAL_POSE_enu_z(buf);
  sample_ev.vel(0) = DL_EXTERNAL_POSE_enu_yd(buf);
  sample_ev.vel(1) = DL_EXTERNAL_POSE_enu_xd(buf);
  sample_ev.vel(2) = -DL_EXTERNAL_POSE_enu_zd(buf);
  sample_ev.quat(0) = DL_EXTERNAL_POSE_body_qi(buf);
  sample_ev.quat(1) = DL_EXTERNAL_POSE_body_qy(buf);
  sample_ev.quat(2) = DL_EXTERNAL_POSE_body_qx(buf);
  sample_ev.quat(3) = -DL_EXTERNAL_POSE_body_qz(buf);

#ifdef INS_EXT_VISION_ROTATION
  // Rotate the quaternion
  struct FloatQuat body_q = {sample_ev.quat(0), sample_ev.quat(1), sample_ev.quat(2), sample_ev.quat(3)};
  struct FloatQuat rot_q;
  float_quat_comp(&rot_q, &body_q, &ins_ext_vision_rot);
  sample_ev.quat(0) = rot_q.qi;
  sample_ev.quat(1) = rot_q.qx;
  sample_ev.quat(2) = rot_q.qy;
  sample_ev.quat(3) = rot_q.qz;
#endif

  // The external pose is given in a north aligned frame (converted from ENU to NED above)
  sample_ev.position_var.setAll(INS_EKF2_EVP_NOISE * INS_EKF2_EVP_NOISE);
  sample_ev.velocity_var.setAll(INS_EKF2_EVV_NOISE * INS_EKF2_EVV_NOISE);
  sample_ev.orientation_var.setAll(INS_EKF2_EVA_NOISE * INS_EKF2_EVA_NOISE);
  sample_ev.pos_frame = PositionFrame::LOCAL_FRAME_NED;
  sample_ev.vel_frame = VelocityFrame::LOCAL_FRAME_NED;
  sample_ev.quality = 100;

  ekf.setExtVisionData(sample_ev);
}

void ins_ekf2_parse_EXTERNAL_POSE_SMALL(uint8_t __attribute__((unused)) *buf)
{

}

/** Publish the attitude and get the new state
 *  Directly called after a succeslfull gyro+accel reading
 */
static void ins_ekf2_publish_attitude(uint64_t time_us)
{
  imuSample imu_sample = {};
  imu_sample.time_us = time_us;
  imu_sample.delta_ang_dt = ekf2.gyro_dt * 1.e-6f;
  imu_sample.delta_ang = Vector3f{ekf2.delta_gyro.p, ekf2.delta_gyro.q, ekf2.delta_gyro.r};
  imu_sample.delta_vel_dt = ekf2.accel_dt * 1.e-6f;
  imu_sample.delta_vel = Vector3f{ekf2.delta_accel.x, ekf2.delta_accel.y, ekf2.delta_accel.z};
  ekf.setIMUData(imu_sample);

  if (ekf.attitude_valid()) {
    // Calculate the quaternion
    struct FloatQuat ltp_to_body_quat;
    const Quatf att_q{ekf.getQuaternion()};
    ltp_to_body_quat.qi = att_q(0);
    ltp_to_body_quat.qx = att_q(1);
    ltp_to_body_quat.qy = att_q(2);
    ltp_to_body_quat.qz = att_q(3);

    // Publish it to the state
    stateSetNedToBodyQuat_f(MODULE_INS_EKF2_ID, &ltp_to_body_quat);

    /* Check the quaternion reset state */
    float delta_q_reset[4];
    uint8_t quat_reset_counter;
    ekf.get_quat_reset(delta_q_reset, &quat_reset_counter);

#ifndef NO_RESET_UPDATE_SETPOINT_HEADING
    // FIXME is this hard reset of control setpoint really needed ? is it the right place ?
    if (ekf2.quat_reset_counter < quat_reset_counter) {
      float psi = matrix::Eulerf(matrix::Quatf(delta_q_reset)).psi();
      guidance_h.sp.heading += psi;
      guidance_h.rc_sp.heading += psi;
      nav.heading += psi;
      //guidance_h_read_rc(autopilot_in_flight());
      stabilization_attitude_enter();
      ekf2.quat_reset_counter = quat_reset_counter;
    }
#endif

    /* Get in-run gyro bias */
    struct FloatRates body_rates;
    Vector3f gyro_bias{ekf.getGyroBias()};
    body_rates.p = (ekf2.delta_gyro.p / (ekf2.gyro_dt * 1.e-6f)) - gyro_bias(0);
    body_rates.q = (ekf2.delta_gyro.q / (ekf2.gyro_dt * 1.e-6f)) - gyro_bias(1);
    body_rates.r = (ekf2.delta_gyro.r / (ekf2.gyro_dt * 1.e-6f)) - gyro_bias(2);

    // Publish it to the state
    stateSetBodyRates_f(MODULE_INS_EKF2_ID, &body_rates);

    /* Get the in-run acceleration bias */
    struct Int32Vect3 accel;
    Vector3f accel_bias{ekf.getAccelBias()};
    accel.x = ACCEL_BFP_OF_REAL((ekf2.delta_accel.x / (ekf2.accel_dt * 1e-6f)) - accel_bias(0));
    accel.y = ACCEL_BFP_OF_REAL((ekf2.delta_accel.y / (ekf2.accel_dt * 1e-6f)) - accel_bias(1));
    accel.z = ACCEL_BFP_OF_REAL((ekf2.delta_accel.z / (ekf2.accel_dt * 1e-6f)) - accel_bias(2));

    // Publish it to the state
    stateSetAccelBody_i(MODULE_INS_EKF2_ID, &accel);
  }

  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = true;
}

/* Update INS based on Baro information */
static void baro_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float pressure)
{
  // Calculate the air density
  float rho = pprz_isa_density_of_pressure(pressure, ekf2.temp);
  ekf.set_air_density(rho);
  ekf2.air_density = rho;

  // Calculate the height above mean sea level based on pressure
  baroSample sample{};
  sample.time_us = ekf2_time_us(stamp);
  sample.hgt = pprz_isa_height_of_pressure_full(pressure, PPRZ_ISA_SEA_LEVEL_PRESSURE); // the EKF estimates the baro offset
  ekf.setBaroData(sample);
}

/* Save the latest temperature measurement for air density calculations */
static void temperature_cb(uint8_t __attribute__((unused)) sender_id, float temp)
{
  ekf2.temp = temp;
}

/* Update INS based on AGL information */
static void agl_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float distance)
{
  estimator::sensor::rangeSample sample{};
  sample.time_us = ekf2_time_us(stamp);
  sample.rng = distance;
  sample.quality = -1; // unknown signal quality

  ekf.setRangeData(sample);
}

/* Update INS based on Gyro information */
static void gyro_int_cb(uint8_t __attribute__((unused)) sender_id,
                        uint32_t stamp, struct FloatRates *delta_gyro, uint16_t dt)
{
  // Copy and save the gyro data
  RATES_COPY(ekf2.delta_gyro, *delta_gyro);
  ekf2.gyro_dt = dt;
  ekf2.gyro_valid = true;

  /* When Gyro and accelerometer are valid enter it into the EKF */
  if (ekf2.gyro_valid && ekf2.accel_valid) {
    ins_ekf2_publish_attitude(ekf2_time_us(stamp));
  }
}

/* Update INS based on Accelerometer information */
static void accel_int_cb(uint8_t sender_id __attribute__((unused)),
                         uint32_t stamp, struct FloatVect3 *delta_accel, uint16_t dt)
{
  // Copy and save the gyro data
  VECT3_COPY(ekf2.delta_accel, *delta_accel);
  ekf2.accel_dt = dt;
  ekf2.accel_valid = true;

  /* When Gyro and accelerometer are valid enter it into the EKF */
  if (ekf2.gyro_valid && ekf2.accel_valid) {
    ins_ekf2_publish_attitude(ekf2_time_us(stamp));
  }
}

/* Update INS based on Magnetometer information */
static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t stamp,
                   struct Int32Vect3 *mag)
{
  struct FloatVect3 mag_gauss;

  // The calibrated magnetometer is normalized: scale it to the field strength of the world
  // magnetic model at the flight plan origin, fixed from the start (a change of the scale
  // while the EKF runs is learned as a magnetometer bias and gives a heading error)
  const float scale = ekf2.mag_scale;
  MAGS_FLOAT_OF_BFP(mag_gauss, *mag);
  mag_gauss.x *= scale;
  mag_gauss.y *= scale;
  mag_gauss.z *= scale;

  // Publish information to the EKF
  magSample sample{};
  sample.time_us = ekf2_time_us(stamp);
  sample.mag = Vector3f{mag_gauss.x, mag_gauss.y, mag_gauss.z};

  ekf.setMagData(sample);
  ekf2.got_imu_data = true;
}

/* Update INS based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
  // The velocity is converted from ECEF when the receiver gives no NED velocity. Receivers
  // without any velocity (e.g. Furuno, NMEA, MTK) are used for the position only, as before.
  bool has_vel = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT) ||
                 bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_ECEF_BIT);
  int32_t gps_ctrl = has_vel ? (int32_t)(INS_EKF2_GPS_CTRL) : ((INS_EKF2_GPS_CTRL) & ~EKF2_GPS_CTRL_VEL);
  if (ekf_params->ekf2_gps_ctrl != gps_ctrl) {
    ekf_params->ekf2_gps_ctrl = gps_ctrl;
  }

  struct LlaCoor_i lla_pos = lla_int_from_gps(gps_s);
  struct NedCoor_f ned_vel = ned_vel_float_from_gps(gps_s);

  // Run the GNSS checks
  gnssChecksSample checks_sample{};
  checks_sample.time_us = ekf2_time_us(stamp);
  checks_sample.lat = lla_pos.lat * 1e-7;
  checks_sample.lon = lla_pos.lon * 1e-7;
  checks_sample.alt = gps_s->hmsl * 1e-3f; // EKF2 works with HMSL
  checks_sample.vel = Vector3f{ned_vel.x, ned_vel.y, ned_vel.z};
  checks_sample.hacc = gps_s->hacc / 100.0f;
  checks_sample.vacc = gps_s->vacc / 100.0f;
  checks_sample.sacc = gps_s->sacc / 100.0f;
  checks_sample.fix_type = gps_s->fix;
  checks_sample.nsats = gps_s->num_sv;
  checks_sample.pdop = gps_s->pdop / 100.0f;
  gnss_checks.run(checks_sample, ekf2.armed, ekf2.in_air, ekf.control_status_flags().vehicle_at_rest);

  // The measurement was taken INS_EKF2_GPS_DELAY before it arrived
  gnssSample sample{};
  sample.time_us = ekf2_delayed_time_us(stamp, INS_EKF2_GPS_DELAY);
  sample.lat = checks_sample.lat;
  sample.lon = checks_sample.lon;
  sample.alt = checks_sample.alt;
  sample.vel = checks_sample.vel;
  sample.hacc = checks_sample.hacc;
  sample.vacc = checks_sample.vacc;
  sample.sacc = checks_sample.sacc;
  sample.fix_type = checks_sample.fix_type;
  sample.nsats = checks_sample.nsats;
  sample.pdop = checks_sample.pdop;
  sample.pos_body = Vector3f{INS_EKF2_GPS_POS_X, INS_EKF2_GPS_POS_Y, INS_EKF2_GPS_POS_Z};
  sample.usable = gnss_checks.passed();
  sample.sacc_passes_strict = !(gnss_checks.getStrictFailFlags() & vehicle_gnss_s::CHECK_SACC);
  ekf.setGpsData(sample);

}

/* Update the heading from the relative position (dual antenna RTK GNSS) */
static void relpos_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, struct RelPosNED *relpos)
{
  // Verify if we received a valid heading
  if (
#ifdef INS_EKF2_RELHEADING_REF_ID
    relpos->reference_id != INS_EKF2_RELHEADING_REF_ID ||
#endif
#ifdef INS_EKF2_GPS_ANTENNA_DISTANCE
    fabs(relpos->distance - INS_EKF2_GPS_ANTENNA_DISTANCE) > INS_EKF2_GPS_ANTENNA_DISTANCE_ERR ||
#endif
    !PX4_ISFINITE(relpos->heading)
  ) {
    return;
  }

#ifdef INS_EKF2_GPS_YAW_OFFSET
  // The heading is measured from antenna 1 to antenna 2, the offset is the yaw of that baseline in the body frame
  gnssYawSample yaw_sample{};
  yaw_sample.time_us = ekf2_delayed_time_us(stamp, INS_EKF2_GPS_DELAY);
  yaw_sample.yaw = wrap_pi(relpos->heading - RadOfDeg(INS_EKF2_GPS_YAW_OFFSET));
  yaw_sample.yaw_acc = (PX4_ISFINITE(relpos->heading_acc) && relpos->heading_acc > 0.f) ? relpos->heading_acc : NAN;
  yaw_sample.yaw_offset = RadOfDeg(INS_EKF2_GPS_YAW_OFFSET);
  yaw_sample.usable = true;
  ekf.setGnssYawData(yaw_sample);
#else
  (void)stamp;
#endif
}

/* Update INS based on Optical Flow information */
static void optical_flow_cb(uint8_t sender_id __attribute__((unused)),
                            uint32_t stamp,
                            int32_t flow_x,
                            int32_t flow_y,
                            int32_t flow_der_x __attribute__((unused)),
                            int32_t flow_der_y __attribute__((unused)),
                            float quality,
                            float size_divergence __attribute__((unused)))
{
  // Wait for two measurements in order to know the integration time
  if (ekf2.flow_stamp <= 0) {
    ekf2.flow_stamp = stamp;
    return;
  }
  uint32_t dt = stamp - ekf2.flow_stamp;
  ekf2.flow_stamp = stamp;

  /* Flow rates of the image about the X and Y body axes (rad/s), RH rotation is positive
  NOTE: pure rotations should result in same flow_x and gyro_roll and same flow_y and gyro_pitch */
  flowSample sample{};
  sample.time_us = ekf2_delayed_time_us(stamp, 0) - dt / 2; // middle of the integration interval
  sample.flow_rate = Vector2f{(float)RadOfDeg(flow_y), -(float)RadOfDeg(flow_x)};
  sample.gyro_rate = Vector3f{NAN, NAN, NAN}; // the EKF uses its own gyro measurements
  sample.quality = quality;                   // quality indicator between 0 and 255

  // Update the optical flow data based on the callback
  ekf.setOpticalFlowData(sample);
}
