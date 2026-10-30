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

/*
 * Options of the previous EKF2 version, still used to configure the EKF.
 * They are converted to the new EKF2 parameters in ins_ekf2_init().
 */
/** Sensors used for fusion (INS_EKF2_FUSION_MODE) */
#define MASK_USE_GPS          (1<<0)  ///< use GNSS position and velocity
#define MASK_USE_OF           (1<<1)  ///< use optical flow
#define MASK_INHIBIT_ACC_BIAS (1<<2)  ///< do not estimate the accelerometer bias
#define MASK_USE_EVPOS        (1<<3)  ///< use external vision position
#define MASK_USE_EVYAW        (1<<4)  ///< use external vision yaw
#define MASK_USE_DRAG         (1<<5)  ///< use the multi-rotor drag model to estimate wind
#define MASK_USE_GPSYAW       (1<<7)  ///< use GNSS yaw (dual antenna)
#define MASK_USE_EVVEL        (1<<8)  ///< use external vision velocity

/** Primary height sensor (INS_EKF2_VDIST_SENSOR_TYPE), same values as the EKF2 height reference */
#define VDIST_SENSOR_BARO  0
#define VDIST_SENSOR_GPS   1
#define VDIST_SENSOR_RANGE 2
#define VDIST_SENSOR_EV    3

/** Magnetometer fusion type (INS_EKF2_MAG_FUSION_TYPE) */
#define MAG_FUSE_TYPE_AUTO    0  ///< automatic heading or 3D fusion
#define MAG_FUSE_TYPE_HEADING 1  ///< always heading fusion
#define MAG_FUSE_TYPE_3D      2  ///< 3D fusion, now part of the automatic selection
#define MAG_FUSE_TYPE_INDOOR  4  ///< magnetometer not used (e.g. indoor with external vision yaw)
#define MAG_FUSE_TYPE_NONE    5  ///< magnetometer not used
#define MAG_FUSE_TYPE_INIT    6  ///< magnetometer only used to initialize the heading

/** GNSS checks (INS_EKF2_GPS_CHECK_MASK), same bits as the PX4 GNSS checks */
#define MASK_GPS_NSATS  (1<<0)  ///< number of satellites
#define MASK_GPS_PDOP   (1<<1)  ///< position dilution of precision
#define MASK_GPS_HACC   (1<<2)  ///< horizontal position accuracy
#define MASK_GPS_VACC   (1<<3)  ///< vertical position accuracy
#define MASK_GPS_SACC   (1<<4)  ///< speed accuracy
#define MASK_GPS_HDRIFT (1<<5)  ///< horizontal drift on the ground
#define MASK_GPS_VDRIFT (1<<6)  ///< vertical drift on the ground
#define MASK_GPS_HSPD   (1<<7)  ///< horizontal speed on the ground
#define MASK_GPS_VSPD   (1<<8)  ///< vertical speed on the ground
#define MASK_GPS_FIX    (1<<10) ///< fix type

/** INS reference from flight plan, true by default */
#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
#endif

/** Special configuration for Optitrack */
#if INS_EKF2_OPTITRACK
#ifndef INS_EKF2_FUSION_MODE
#define INS_EKF2_FUSION_MODE (MASK_USE_EVPOS | MASK_USE_EVVEL | MASK_USE_EVYAW)
#endif
#ifndef INS_EKF2_MAG_FUSION_TYPE
#define INS_EKF2_MAG_FUSION_TYPE MAG_FUSE_TYPE_INDOOR
#endif
#ifndef INS_EKF2_VDIST_SENSOR_TYPE
#define INS_EKF2_VDIST_SENSOR_TYPE VDIST_SENSOR_EV
#endif
#endif

/** The EKF2 fusion mode setting */
#ifndef INS_EKF2_FUSION_MODE
#define INS_EKF2_FUSION_MODE (MASK_USE_GPS)
#endif
PRINT_CONFIG_VAR(INS_EKF2_FUSION_MODE)

/** The EKF2 magnetometer fusion type */
#ifndef INS_EKF2_MAG_FUSION_TYPE
#define INS_EKF2_MAG_FUSION_TYPE MAG_FUSE_TYPE_AUTO
#endif
PRINT_CONFIG_VAR(INS_EKF2_MAG_FUSION_TYPE)

/** The EKF2 primary vertical distance sensor type */
#ifndef INS_EKF2_VDIST_SENSOR_TYPE
#define INS_EKF2_VDIST_SENSOR_TYPE VDIST_SENSOR_BARO
#endif
PRINT_CONFIG_VAR(INS_EKF2_VDIST_SENSOR_TYPE)

/** The GNSS checks before the GNSS data is used, a 3D fix is always required
 *  (the default thresholds below are the ones of the previous EKF2 version) */
#ifndef INS_EKF2_GPS_CHECK_MASK
#define INS_EKF2_GPS_CHECK_MASK (MASK_GPS_NSATS | MASK_GPS_HACC | MASK_GPS_SACC)
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_CHECK_MASK)

/** Required number of satellites for the GNSS checks */
#ifndef INS_EKF2_GPS_REQ_NSATS
#define INS_EKF2_GPS_REQ_NSATS 6
#endif

/** Maximum horizontal position accuracy for the GNSS checks (m) */
#ifndef INS_EKF2_GPS_REQ_EPH
#define INS_EKF2_GPS_REQ_EPH 5.0f
#endif

/** Maximum vertical position accuracy for the GNSS checks (m) */
#ifndef INS_EKF2_GPS_REQ_EPV
#define INS_EKF2_GPS_REQ_EPV 8.0f
#endif

/** Maximum speed accuracy for the GNSS checks (m/s) */
#ifndef INS_EKF2_GPS_REQ_SACC
#define INS_EKF2_GPS_REQ_SACC 1.0f
#endif

/** Maximum PDOP for the GNSS checks */
#ifndef INS_EKF2_GPS_REQ_PDOP
#define INS_EKF2_GPS_REQ_PDOP 2.0f
#endif

/** GNSS measurement delay (ms) */
#ifndef INS_EKF2_GPS_DELAY
#define INS_EKF2_GPS_DELAY 110
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_DELAY)

/** External vision measurement delay (ms) */
#ifndef INS_EKF2_EV_DELAY
#define INS_EKF2_EV_DELAY 175
#endif
PRINT_CONFIG_VAR(INS_EKF2_EV_DELAY)

/** Default AGL sensor minimum range */
#ifndef INS_EKF2_SONAR_MIN_RANGE
#define INS_EKF2_SONAR_MIN_RANGE 0.001
#endif
PRINT_CONFIG_VAR(INS_EKF2_SONAR_MIN_RANGE)

/** Default AGL sensor maximum range */
#ifndef INS_EKF2_SONAR_MAX_RANGE
#define INS_EKF2_SONAR_MAX_RANGE 4
#endif
PRINT_CONFIG_VAR(INS_EKF2_SONAR_MAX_RANGE)

/** If enabled uses radar sensor as primary AGL source, if possible */
#ifndef INS_EKF2_RANGE_MAIN_AGL
#define INS_EKF2_RANGE_MAIN_AGL 1
#endif
PRINT_CONFIG_VAR(INS_EKF2_RANGE_MAIN_AGL)

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

/* IMU X offset from CoG position in meters */
#ifndef INS_EKF2_IMU_POS_X
#define INS_EKF2_IMU_POS_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_IMU_POS_X)

/* IMU Y offset from CoG position in meters */
#ifndef INS_EKF2_IMU_POS_Y
#define INS_EKF2_IMU_POS_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_IMU_POS_Y)

/* IMU Z offset from CoG position in meters */
#ifndef INS_EKF2_IMU_POS_Z
#define INS_EKF2_IMU_POS_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_IMU_POS_Z)

/* GPS X offset from CoG position in meters */
#ifndef INS_EKF2_GPS_POS_X
#define INS_EKF2_GPS_POS_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_X)

/* GPS Y offset from CoG position in meters */
#ifndef INS_EKF2_GPS_POS_Y
#define INS_EKF2_GPS_POS_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_Y)

/* GPS Z offset from CoG position in meters */
#ifndef INS_EKF2_GPS_POS_Z
#define INS_EKF2_GPS_POS_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_Z)

/* Default flow/radar message delay (in ms) */
#ifndef INS_EKF2_FLOW_SENSOR_DELAY
#define INS_EKF2_FLOW_SENSOR_DELAY 15
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_SENSOR_DELAY)

/* Default minimum accepted quality (1 to 255) */
#ifndef INS_EKF2_MIN_FLOW_QUALITY
#define INS_EKF2_MIN_FLOW_QUALITY  100
#endif
PRINT_CONFIG_VAR(INS_EKF2_MIN_FLOW_QUALITY)

/* Max flow rate that the sensor can measure (rad/sec) */
#ifndef INS_EKF2_MAX_FLOW_RATE
#define INS_EKF2_MAX_FLOW_RATE 200
#endif
PRINT_CONFIG_VAR(INS_EKF2_MAX_FLOW_RATE)

/* Flow sensor X offset from CoG position in meters */
#ifndef INS_EKF2_FLOW_POS_X
#define INS_EKF2_FLOW_POS_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_POS_X)

/* Flow sensor Y offset from CoG position in meters */
#ifndef INS_EKF2_FLOW_POS_Y
#define INS_EKF2_FLOW_POS_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_POS_Y)

/* Flow sensor Z offset from CoG position in meters */
#ifndef INS_EKF2_FLOW_POS_Z
#define INS_EKF2_FLOW_POS_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_POS_Z)

/* Flow sensor noise in rad/sec */
#ifndef INS_EKF2_FLOW_NOISE
#define INS_EKF2_FLOW_NOISE 0.03
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_NOISE)

/* Flow sensor noise at qmin in rad/sec */
#ifndef INS_EKF2_FLOW_NOISE_QMIN
#define INS_EKF2_FLOW_NOISE_QMIN 0.05
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_NOISE_QMIN)

/* Flow sensor innovation gate */
#ifndef INS_EKF2_FLOW_INNOV_GATE
#define INS_EKF2_FLOW_INNOV_GATE 4
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_INNOV_GATE)

/* External vision position noise (m) */
#ifndef INS_EKF2_EVP_NOISE
#define INS_EKF2_EVP_NOISE 0.02f
#endif
PRINT_CONFIG_VAR(INS_EKF2_EVP_NOISE)

/* External vision velocity noise (m/s) */
#ifndef INS_EKF2_EVV_NOISE
#define INS_EKF2_EVV_NOISE 0.1f
#endif
PRINT_CONFIG_VAR(INS_EKF2_EVV_NOISE)

/* External vision angle noise (rad) */
#ifndef INS_EKF2_EVA_NOISE
#define INS_EKF2_EVA_NOISE 0.05f
#endif
PRINT_CONFIG_VAR(INS_EKF2_EVA_NOISE)

/* GPS measurement noise for horizontal velocity (m/s) */
#ifndef INS_EKF2_GPS_V_NOISE
#define INS_EKF2_GPS_V_NOISE 0.3f
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_V_NOISE)

/* GPS measurement position noise (m) */
#ifndef INS_EKF2_GPS_P_NOISE
#define INS_EKF2_GPS_P_NOISE 0.5f
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_P_NOISE)

/* Barometric measurement noise for altitude (m) */
#ifndef INS_EKF2_BARO_NOISE
#define INS_EKF2_BARO_NOISE 3.5f
#endif
PRINT_CONFIG_VAR(INS_EKF2_BARO_NOISE)

/* GNSS heading measurement noise when the receiver gives no accuracy (rad) */
#ifndef INS_EKF2_GPS_YAW_NOISE
#define INS_EKF2_GPS_YAW_NOISE 0.1f
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_YAW_NOISE)

/* Maximum allowed distance error for the RTK relative heading measurement (m) */
#ifndef INS_EKF2_RELHEADING_ERR
#define INS_EKF2_RELHEADING_ERR 0.2
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
static void reset_cb(uint8_t sender_id, uint8_t flag);

/* Static local functions */
static void ins_ekf2_publish_attitude(uint64_t time_us);
static void ins_ekf2_set_gps_ctrl(int32_t fusion_mode);

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

  // A stamp later than now was taken before the last wrap
  uint64_t time_us = wraps + stamp;
  if (stamp > now && time_us >= (1ULL << 32)) {
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

  pprz_msg_send_INS_EKF2_EXT(trans, dev, AC_ID,
                             &gps_drift[0], &gps_drift[1], &gps_drift[2], &gps_blocked,
                             &ekf2.vibe[0], &ekf2.vibe[1], &ekf2.vibe[2]);
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

/** Convert the magnetometer fusion type of the previous EKF2 version */
static int32_t mag_type_of_fusion_type(int32_t fusion_type)
{
  switch (fusion_type) {
    case MAG_FUSE_TYPE_HEADING:
      return static_cast<int32_t>(MagFuseType::HEADING);
    case MAG_FUSE_TYPE_INDOOR:
    case MAG_FUSE_TYPE_NONE:
      return static_cast<int32_t>(MagFuseType::NONE);
    case MAG_FUSE_TYPE_INIT:
      return static_cast<int32_t>(MagFuseType::INIT);
    case MAG_FUSE_TYPE_3D:
    case MAG_FUSE_TYPE_AUTO:
    default:
      return static_cast<int32_t>(MagFuseType::AUTO);
  }
}

/** Set the GNSS fusion from the fusion mode (also used to remove GNSS in flight) */
static void ins_ekf2_set_gps_ctrl(int32_t fusion_mode)
{
  int32_t gps_ctrl = 0;
  if (fusion_mode & MASK_USE_GPS) {
    gps_ctrl |= static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL);
    if (INS_EKF2_VDIST_SENSOR_TYPE == VDIST_SENSOR_GPS) {
      gps_ctrl |= static_cast<int32_t>(GnssCtrl::VPOS);
    }
  }
  if (fusion_mode & MASK_USE_GPSYAW) {
    gps_ctrl |= static_cast<int32_t>(GnssCtrl::YAW);
  }
  ekf_params->ekf2_gps_ctrl = gps_ctrl;
}

/* Initialize the EKF */
void ins_ekf2_init(void)
{
  /* Get the ekf parameters */
  ekf_params = ekf.getParamHandle();

  /* Sensors used for fusion */
  ekf2.fusion_mode = 0;
  ins_ekf2_set_gps_ctrl(INS_EKF2_FUSION_MODE);
  ekf_params->ekf2_hgt_ref = INS_EKF2_VDIST_SENSOR_TYPE;
  ekf_params->ekf2_baro_ctrl = 1;
  ekf_params->ekf2_mag_type = mag_type_of_fusion_type(INS_EKF2_MAG_FUSION_TYPE);
  ekf_params->ekf2_of_ctrl = ((INS_EKF2_FUSION_MODE) & MASK_USE_OF) ? 1 : 0;
  ekf_params->ekf2_drag_ctrl = ((INS_EKF2_FUSION_MODE) & MASK_USE_DRAG) ? 1 : 0;
  ekf_params->ekf2_ev_ctrl = (((INS_EKF2_FUSION_MODE) & MASK_USE_EVPOS) ? static_cast<int32_t>(EvCtrl::HPOS) : 0)
                             | ((INS_EKF2_VDIST_SENSOR_TYPE == VDIST_SENSOR_EV) ? static_cast<int32_t>(EvCtrl::VPOS) : 0)
                             | (((INS_EKF2_FUSION_MODE) & MASK_USE_EVVEL) ? static_cast<int32_t>(EvCtrl::VEL) : 0)
                             | (((INS_EKF2_FUSION_MODE) & MASK_USE_EVYAW) ? static_cast<int32_t>(EvCtrl::YAW) : 0);
  if ((INS_EKF2_FUSION_MODE) & MASK_INHIBIT_ACC_BIAS) {
    ekf_params->ekf2_imu_ctrl &= ~static_cast<int32_t>(ImuCtrl::AccelBias);
  }

  /* Range finder as height source: always when it is the primary height sensor, else when suitable */
  if (INS_EKF2_VDIST_SENSOR_TYPE == VDIST_SENSOR_RANGE) {
    ekf_params->ekf2_rng_ctrl = static_cast<int32_t>(RngCtrl::ENABLED);
  } else {
    ekf_params->ekf2_rng_ctrl = static_cast<int32_t>(INS_EKF2_RANGE_MAIN_AGL ? RngCtrl::CONDITIONAL : RngCtrl::DISABLED);
  }

  /* Set specific noise levels */
  ekf_params->ekf2_acc_b_noise = 3.0e-3f;
  ekf_params->ekf2_gps_v_noise = INS_EKF2_GPS_V_NOISE;
  ekf_params->ekf2_gps_p_noise = INS_EKF2_GPS_P_NOISE;
  ekf_params->ekf2_baro_noise = INS_EKF2_BARO_NOISE;
  ekf_params->gnss_heading_noise = INS_EKF2_GPS_YAW_NOISE;
  ekf_params->ekf2_evp_noise = INS_EKF2_EVP_NOISE;
  ekf_params->ekf2_evv_noise = INS_EKF2_EVV_NOISE;
  ekf_params->ekf2_eva_noise = INS_EKF2_EVA_NOISE;
  ekf_params->ekf2_ev_delay = INS_EKF2_EV_DELAY;

  /* Set optical flow and range finder parameters */
  ekf_params->ekf2_of_qmin = INS_EKF2_MIN_FLOW_QUALITY;
  ekf_params->ekf2_of_delay = INS_EKF2_FLOW_SENSOR_DELAY;
  ekf_params->ekf2_rng_delay = INS_EKF2_FLOW_SENSOR_DELAY;
  ekf_params->ekf2_of_n_min = INS_EKF2_FLOW_NOISE;
  ekf_params->ekf2_of_n_max = INS_EKF2_FLOW_NOISE_QMIN;
  ekf_params->ekf2_of_gate = INS_EKF2_FLOW_INNOV_GATE;

  /* The observation buffers must hold the longest delay of the used sensors */
  float delay_max = math::max((float)INS_EKF2_FLOW_SENSOR_DELAY, (float)INS_EKF2_GPS_DELAY);
  if (ekf_params->ekf2_ev_ctrl != 0) {
    delay_max = math::max(delay_max, (float)INS_EKF2_EV_DELAY);
  }
  ekf_params->ekf2_delay_max = delay_max;

  /* Set the IMU position relative from the CoG in xyz (m) */
  ekf_params->imu_pos_body = {
    INS_EKF2_IMU_POS_X,
    INS_EKF2_IMU_POS_Y,
    INS_EKF2_IMU_POS_Z
  };
  ekf.output_predictor().set_imu_offset(ekf_params->imu_pos_body);

  /* Set flow sensor offset from CoG position in xyz (m) */
  ekf_params->flow_pos_body = {
    INS_EKF2_FLOW_POS_X,
    INS_EKF2_FLOW_POS_Y,
    INS_EKF2_FLOW_POS_Z
  };

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

  /* GNSS checks before the GNSS data is used (a 3D fix is always required) */
  GnssChecks::Params checks_params{};
  checks_params.check_mask = INS_EKF2_GPS_CHECK_MASK | MASK_GPS_FIX;
  checks_params.req_nsats = INS_EKF2_GPS_REQ_NSATS;
  checks_params.req_pdop = INS_EKF2_GPS_REQ_PDOP;
  checks_params.req_eph = INS_EKF2_GPS_REQ_EPH;
  checks_params.req_epv = INS_EKF2_GPS_REQ_EPV;
  checks_params.req_sacc = INS_EKF2_GPS_REQ_SACC;
  checks_params.req_fix = 3;
  gnss_checks.setParams(checks_params);

  /* Initialize struct */
  ekf2.ltp_stamp = 0;
  ekf2.flow_stamp = 0;
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = false;
  ekf2.quat_reset_counter = 0;
  ekf2.temp = 20.0f; // Default temperature of 20 degrees celcius
  ekf2.qnh = 1013.25f; // Default atmosphere
  ekf2.in_air = false;
  ekf2.armed = false;
  ekf2.system_flags_time = 0;
  FLOAT_VECT3_ZERO(ekf2.accel_ned);

  /* Initialize the range sensor limits */
  ekf.set_rangefinder_limits(INS_EKF2_SONAR_MIN_RANGE, INS_EKF2_SONAR_MAX_RANGE);

  /* Initialize the flow sensor limits */
  ekf.set_optical_flow_limits(INS_EKF2_MAX_FLOW_RATE, INS_EKF2_SONAR_MIN_RANGE, INS_EKF2_SONAR_MAX_RANGE);

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
  uint64_t now = ekf2_time_us(get_sys_time_usec());
  if (in_air == ekf2.in_air && armed == ekf2.armed && now < ekf2.system_flags_time + 1000000) {
    return;
  }

  systemFlagUpdate flags{};
  flags.time_us = now;
  flags.armed = armed;
  flags.in_air = in_air;
#if defined(FIXEDWING_FIRMWARE)
  flags.is_fixed_wing = true;
#endif
  ekf.setSystemFlagData(flags);

  ekf2.in_air = in_air;
  ekf2.armed = armed;
  ekf2.system_flags_time = now;
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

/** Remove the GNSS position and velocity from the fusion and use optical flow and GNSS heading instead */
void ins_ekf2_remove_gps(int32_t mode)
{
  ekf2.fusion_mode = mode;
  if (mode) {
    ins_ekf2_set_gps_ctrl(MASK_USE_OF | MASK_USE_GPSYAW);
    ekf_params->ekf2_of_ctrl = 1;
  } else {
    ins_ekf2_set_gps_ctrl(INS_EKF2_FUSION_MODE);
    ekf_params->ekf2_of_ctrl = ((INS_EKF2_FUSION_MODE) & MASK_USE_OF) ? 1 : 0;
  }
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

/** Update the IMU vibration metrics (coning, high frequency gyro and accelerometer vibration) */
static void ins_ekf2_update_vibration(const Vector3f &delta_ang, const Vector3f &delta_vel)
{
  const Vector3f prev_ang(ekf2.prev_delta_gyro.p, ekf2.prev_delta_gyro.q, ekf2.prev_delta_gyro.r);
  const Vector3f prev_vel(ekf2.prev_delta_accel.x, ekf2.prev_delta_accel.y, ekf2.prev_delta_accel.z);
  const float alpha = 0.01f;
  ekf2.vibe[0] = (1.f - alpha) * ekf2.vibe[0] + alpha * delta_ang.cross(prev_ang).norm();
  ekf2.vibe[1] = (1.f - alpha) * ekf2.vibe[1] + alpha * (delta_ang - prev_ang).norm();
  ekf2.vibe[2] = (1.f - alpha) * ekf2.vibe[2] + alpha * (delta_vel - prev_vel).norm();
  RATES_ASSIGN(ekf2.prev_delta_gyro, delta_ang(0), delta_ang(1), delta_ang(2));
  VECT3_ASSIGN(ekf2.prev_delta_accel, delta_vel(0), delta_vel(1), delta_vel(2));
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
  ins_ekf2_update_vibration(imu_sample.delta_ang, imu_sample.delta_vel);

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

  // Calculate the height above mean sea level based on pressure
  baroSample sample{};
  sample.time_us = ekf2_time_us(stamp);
  sample.hgt = pprz_isa_height_of_pressure_full(pressure, ekf2.qnh * 100.0f);
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

  // Convert Magnetometer information to float and to radius 0.2f
  MAGS_FLOAT_OF_BFP(mag_gauss, *mag);
  mag_gauss.x *= 0.4f;
  mag_gauss.y *= 0.4f;
  mag_gauss.z *= 0.4f;

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
  // The EKF needs the NED velocity
  if (!bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT)) {
    return;
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
  gnss_checks.run(checks_sample, ekf2.armed, ekf2.in_air, false);

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

#if INS_EKF2_GPS_COURSE_YAW
  // Use the course over ground as heading
  gnssYawSample yaw_sample{};
  yaw_sample.time_us = sample.time_us;
  yaw_sample.yaw = wrap_pi((float)gps_s->course / 1e7);
  yaw_sample.yaw_acc = NAN;
  yaw_sample.yaw_offset = 0;
  yaw_sample.usable = true;
  ekf.setGnssYawData(yaw_sample);
#endif
}

/* Update the heading from the relative position (dual antenna RTK GNSS) */
static void relpos_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, struct RelPosNED *relpos)
{
  // Verify if we received a valid heading
  if (
#ifdef INS_EKF2_RELHEADING_REF_ID
    relpos->reference_id != INS_EKF2_RELHEADING_REF_ID ||
#endif
#ifdef INS_EKF2_RELHEADING_DISTANCE
    fabs(relpos->distance - INS_EKF2_RELHEADING_DISTANCE) > INS_EKF2_RELHEADING_ERR ||
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
