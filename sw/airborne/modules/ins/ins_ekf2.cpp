/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 */

#include "modules/ins/ins_ekf2.h"
#include "modules/nav/waypoints.h"
#include "modules/core/abi.h"
#include "stabilization/stabilization_attitude.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "EKF/ekf.h"
#include "math/pprz_isa.h"
#include "math/pprz_geodetic_wgs84.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"

/** For SITL and NPS we need special includes */
#if defined SITL && USE_NPS
#include "nps_autopilot.h"
#include <stdio.h>
#endif

/** Prevent setting INS reference from flight plan */
#if USE_INS_NAV_INIT
#error INS initialization from flight plan is not yet supported
#endif

/** Special configuration for Optitrack */
#if INS_EKF2_OPTITRACK
#ifndef INS_EKF2_FUSION_MODE
#define INS_EKF2_FUSION_MODE (MASK_USE_EVPOS | MASK_USE_EVVEL | MASK_USE_EVYAW)
#endif
#ifndef INS_EKF2_VDIST_SENSOR_TYPE
#define INS_EKF2_VDIST_SENSOR_TYPE VDIST_SENSOR_EV
#endif
#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT true
#endif
#endif

/** The EKF2 fusion mode setting */
#ifndef INS_EKF2_FUSION_MODE
#define INS_EKF2_FUSION_MODE (MASK_USE_GPS)
#endif
PRINT_CONFIG_VAR(INS_EKF2_FUSION_MODE)

/** The EKF2 primary vertical distance sensor type */
#ifndef INS_EKF2_VDIST_SENSOR_TYPE
#define INS_EKF2_VDIST_SENSOR_TYPE VDIST_SENSOR_BARO
#endif
PRINT_CONFIG_VAR(INS_EKF2_VDIST_SENSOR_TYPE)

/** The EKF2 GPS checks before initialization */
#ifndef INS_EKF2_GPS_CHECK_MASK
#define INS_EKF2_GPS_CHECK_MASK 21 // (MASK_GPS_NSATS | MASK_GPS_HACC | MASK_GPS_SACC)
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_CHECK_MASK)

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
PRINT_CONFIG_VAR(INS_FLOW_SENSOR_DELAY)

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

/* All registered ABI events */
static abi_event baro_ev;
static abi_event temperature_ev;
static abi_event agl_ev;
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event gps_ev;
static abi_event body_to_imu_ev;
static abi_event optical_flow_ev;

/* All ABI callbacks */
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);
static void temperature_cb(uint8_t sender_id, float temp);
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static void optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x, int32_t flow_y, int32_t flow_der_x,
                            int32_t flow_der_y, float quality, float size_divergence);

/* Static local functions */
static void ins_ekf2_publish_attitude(uint32_t stamp);

/* Static local variables */
static Ekf ekf;                                   ///< EKF class itself
static parameters *ekf_params;                    ///< The EKF parameters
struct ekf2_t ekf2;                               ///< Local EKF2 status structure

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i pos, speed, accel;

  // Get it from the EKF
  const Vector3f pos_f{ekf.getPosition()};
  const Vector3f speed_f{ekf.getVelocity()};
  const Vector3f accel_f{ekf.getVelocityDerivative()};

  // Convert to integer
  pos.x = POS_BFP_OF_REAL(pos_f(0));
  pos.y = POS_BFP_OF_REAL(pos_f(1));
  pos.z = POS_BFP_OF_REAL(pos_f(2));
  speed.x = SPEED_BFP_OF_REAL(speed_f(0));
  speed.y = SPEED_BFP_OF_REAL(speed_f(1));
  speed.z = SPEED_BFP_OF_REAL(speed_f(2));
  accel.x = ACCEL_BFP_OF_REAL(accel_f(0));
  accel.y = ACCEL_BFP_OF_REAL(accel_f(1));
  accel.z = ACCEL_BFP_OF_REAL(accel_f(2));

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
  const Vector3f accel_f{ekf.getVelocityDerivative()};

  // Convert to integer
  pos_z = POS_BFP_OF_REAL(pos_f(2));
  speed_z = SPEED_BFP_OF_REAL(speed_f(2));
  accel_z = ACCEL_BFP_OF_REAL(accel_f(2));

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

static void send_ins_ekf2(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t gps_check_status, soln_status;
  uint16_t filter_fault_status = ekf.fault_status().value; // FIXME: 32bit instead of 16bit
  uint32_t control_mode = ekf.control_status().value;
  ekf.get_gps_check_status(&gps_check_status);
  ekf.get_ekf_soln_status(&soln_status);

  uint16_t innov_test_status;
  float mag, vel, pos, hgt, tas, hagl, flow, beta, mag_decl;
  uint8_t terrain_valid, dead_reckoning;
  ekf.get_innovation_test_status(innov_test_status, mag, vel, pos, hgt, tas, hagl, beta);
  //ekf.get_flow_innov(&flow);
  ekf.get_mag_decl_deg(&mag_decl);

  if (ekf.isTerrainEstimateValid()) {
    terrain_valid = 1;
  } else {
    terrain_valid = 0;
  }

  if (ekf.inertial_dead_reckoning()) {
    dead_reckoning = 1;
  } else {
    dead_reckoning = 0;
  }

  pprz_msg_send_INS_EKF2(trans, dev, AC_ID,
                         &control_mode, &filter_fault_status, &gps_check_status, &soln_status,
                         &innov_test_status, &mag, &vel, &pos, &hgt, &tas, &hagl, &flow, &beta,
                         &mag_decl, &terrain_valid, &dead_reckoning);
}

static void send_ins_ekf2_ext(struct transport_tx *trans, struct link_device *dev)
{
  float gps_drift[3];
  Vector3f vibe = ekf.getImuVibrationMetrics();
  bool gps_blocked;
  uint8_t gps_blocked_b;
  ekf.get_gps_drift_metrics(gps_drift, &gps_blocked);
  gps_blocked_b = gps_blocked;

  pprz_msg_send_INS_EKF2_EXT(trans, dev, AC_ID,
                             &gps_drift[0], &gps_drift[1], &gps_drift[2], &gps_blocked_b,
                             &vibe(0), &vibe(1), &vibe(2));
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t ahrs_ekf2_id = AHRS_COMP_ID_EKF2;
  filter_control_status_u control_mode = ekf.control_status();
  uint32_t filter_fault_status = ekf.fault_status().value;
  uint16_t filter_fault_status_16 = filter_fault_status; //FIXME
  uint8_t mde = 0;

  // Check the alignment and if GPS is fused
  if (control_mode.flags.tilt_align && control_mode.flags.yaw_align && control_mode.flags.gps) {
    mde = 3;
  } else if (control_mode.flags.tilt_align && control_mode.flags.yaw_align) {
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
  float tas;
  Vector2f wind = ekf.getWindVelocity();
  uint8_t flags = 0x5;
  float f_zero = 0;

  ekf.get_true_airspeed(&tas);

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
#endif

/* Initialize the EKF */
void ins_ekf2_init(void)
{
  /* Get the ekf parameters */
  ekf_params = ekf.getParamHandle();
  ekf_params->fusion_mode = INS_EKF2_FUSION_MODE;
  ekf_params->vdist_sensor_type = INS_EKF2_VDIST_SENSOR_TYPE;
  ekf_params->gps_check_mask = INS_EKF2_GPS_CHECK_MASK;

  /* Set specific noise levels */
  ekf_params->accel_bias_p_noise = 3.0e-3f;
  ekf_params->gps_vel_noise = INS_EKF2_GPS_V_NOISE;
  ekf_params->gps_pos_noise = INS_EKF2_GPS_P_NOISE;
  ekf_params->baro_noise = INS_EKF2_BARO_NOISE;

  /* Set optical flow parameters */
  ekf_params->flow_qual_min = INS_EKF2_MIN_FLOW_QUALITY;
  ekf_params->flow_delay_ms = INS_EKF2_FLOW_SENSOR_DELAY;
  ekf_params->range_delay_ms = INS_EKF2_FLOW_SENSOR_DELAY;
  ekf_params->flow_noise = INS_EKF2_FLOW_NOISE;
  ekf_params->flow_noise_qual_min = INS_EKF2_FLOW_NOISE_QMIN;
  ekf_params->flow_innov_gate = INS_EKF2_FLOW_INNOV_GATE;

  /* Set the IMU position relative from the CoG in xyz (m) */
  ekf_params->imu_pos_body = {
    INS_EKF2_IMU_POS_X,
    INS_EKF2_IMU_POS_Y,
    INS_EKF2_IMU_POS_Z
  };

  /* Set the GPS position relative from the CoG in xyz (m) */
  ekf_params->gps_pos_body = {
    INS_EKF2_GPS_POS_X,
    INS_EKF2_GPS_POS_Y,
    INS_EKF2_GPS_POS_Z
  };

  /* Set flow sensor offset from CoG position in xyz (m) */
  ekf_params->flow_pos_body = {
    INS_EKF2_FLOW_POS_X,
    INS_EKF2_FLOW_POS_Y,
    INS_EKF2_FLOW_POS_Z
  };

  /* Set range as default AGL measurement if possible */
  ekf_params->range_aid = INS_EKF2_RANGE_MAIN_AGL;

  /* Initialize struct */
  ekf2.ltp_stamp = 0;
  ekf2.accel_stamp = 0;
  ekf2.gyro_stamp = 0;
  ekf2.flow_stamp = 0;
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = false;
  ekf2.quat_reset_counter = 0;
  ekf2.temp = 20.0f; // Default temperature of 20 degrees celcius
  ekf2.qnh = 1013.25f; // Default atmosphere

  /* Initialize the range sensor limits */
  ekf.set_rangefinder_limits(INS_EKF2_SONAR_MIN_RANGE, INS_EKF2_SONAR_MAX_RANGE);

  /* Initialize the flow sensor limits */
  ekf.set_optical_flow_limits(INS_EKF2_MAX_FLOW_RATE, INS_EKF2_SONAR_MIN_RANGE, INS_EKF2_SONAR_MAX_RANGE);

  /* Initialize the origin from flight plan */
#if USE_INS_NAV_INIT
  if(ekf.setEkfGlobalOrigin(NAV_LAT0*1e-7, NAV_LON0*1e-7, (NAV_ALT0 + NAV_MSL0)*1e-3))
  {
    struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
    llh_nav0.lat = NAV_LAT0;
    llh_nav0.lon = NAV_LON0;
    /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
    llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

    ltp_def_from_lla_i(&ekf2.ltp_def, &llh_nav0);
    ekf2.ltp_def.hmsl = NAV_ALT0;
    stateSetLocalOrigin_i(&ekf2.ltp_def);

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
#endif

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgBARO_ABS(INS_EKF2_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgTEMPERATURE(INS_EKF2_TEMPERATURE_ID, &temperature_ev, temperature_cb);
  AbiBindMsgAGL(INS_EKF2_AGL_ID, &agl_ev, agl_cb);
  AbiBindMsgIMU_GYRO(INS_EKF2_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(INS_EKF2_ACCEL_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG(INS_EKF2_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgGPS(INS_EKF2_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgOPTICAL_FLOW(INS_EKF2_OF_ID, &optical_flow_ev, optical_flow_cb);
}

/* Update the INS state */
void ins_ekf2_update(void)
{
  /* Set EKF settings */
  ekf.set_in_air_status(autopilot_in_flight());

  /* Update the EKF */
  if (ekf2.got_imu_data) {
    // Update the EKF but ignore the response and also copy the faster intermediate filter
    ekf.update();
    filter_control_status_u control_status = ekf.control_status();

    // Only publish position after successful alignment
    if (control_status.flags.tilt_align) {
      /* Get the position */
      const Vector3f pos_f{ekf.getPosition()};
      struct NedCoor_f pos;
      pos.x = pos_f(0);
      pos.y = pos_f(1);
      pos.z = pos_f(2);

      // Publish to the state
      stateSetPositionNed_f(&pos);

      /* Get the velocity in NED frame */
      const Vector3f vel_f{ekf.getVelocity()};
      struct NedCoor_f speed;
      speed.x = vel_f(0);
      speed.y = vel_f(1);
      speed.z = vel_f(2);

      // Publish to state
      stateSetSpeedNed_f(&speed);

      /* Get the accelerations in NED frame */
      const Vector3f vel_deriv_f{ekf.getVelocityDerivative()};
      struct NedCoor_f accel;
      accel.x = vel_deriv_f(0);
      accel.y = vel_deriv_f(1);
      accel.z = vel_deriv_f(2);

      // Publish to state
      stateSetAccelNed_f(&accel);

      /* Get local origin */
      // Position of local NED origin in GPS / WGS84 frame
      double ekf_origin_lat, ekf_origin_lon;
      float ref_alt;
      struct LlaCoor_i lla_ref;
      uint64_t origin_time;

      // Only update the origin when the state estimator has updated the origin
      bool ekf_origin_valid = ekf.getEkfGlobalOrigin(origin_time, ekf_origin_lat, ekf_origin_lon, ref_alt);
      if (ekf_origin_valid && (origin_time > ekf2.ltp_stamp)) {
        lla_ref.lat = ekf_origin_lat * 1e7; // WGS-84 lat
        lla_ref.lon = ekf_origin_lon * 1e7; // WGS-84 lon
        lla_ref.alt = ref_alt * 1e3 + wgs84_ellipsoid_to_geoid_i(lla_ref.lat, lla_ref.lon); // WGS-84 height
        ltp_def_from_lla_i(&ekf2.ltp_def, &lla_ref);
        ekf2.ltp_def.hmsl = ref_alt * 1e3;
        stateSetLocalOrigin_i(&ekf2.ltp_def);

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

void ins_ekf2_change_param(int32_t unk)
{
  ekf_params->mag_fusion_type = ekf2.mag_fusion_type = unk;
}

void ins_ekf2_remove_gps(int32_t mode)
{
  if (mode) {
    ekf_params->fusion_mode = ekf2.fusion_mode = (MASK_USE_OF | MASK_USE_GPSYAW);
  } else {
    ekf_params->fusion_mode = ekf2.fusion_mode = INS_EKF2_FUSION_MODE;
  }
}

void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf) {
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  extVisionSample sample;
  sample.time_us = get_sys_time_usec(); //FIXME
  sample.pos(0) = DL_EXTERNAL_POSE_enu_y(buf);
  sample.pos(1) = DL_EXTERNAL_POSE_enu_x(buf);
  sample.pos(2) = -DL_EXTERNAL_POSE_enu_z(buf);
  sample.vel(0) = DL_EXTERNAL_POSE_enu_yd(buf);
  sample.vel(1) = DL_EXTERNAL_POSE_enu_xd(buf);
  sample.vel(2) = -DL_EXTERNAL_POSE_enu_zd(buf);
  sample.quat(0) = DL_EXTERNAL_POSE_body_qi(buf);
  sample.quat(1) = DL_EXTERNAL_POSE_body_qy(buf);
  sample.quat(2) = DL_EXTERNAL_POSE_body_qx(buf);
  sample.quat(3) = -DL_EXTERNAL_POSE_body_qz(buf);
  sample.posVar.setAll(INS_EKF2_EVP_NOISE);
  sample.velCov = matrix::eye<float, 3>() * INS_EKF2_EVV_NOISE;
  sample.angVar = INS_EKF2_EVA_NOISE;
  sample.vel_frame = velocity_frame_t::LOCAL_FRAME_FRD;

  ekf.setExtVisionData(sample);
}

void ins_ekf2_parse_EXTERNAL_POSE_SMALL(uint8_t __attribute__((unused)) *buf) {

}

/** Publish the attitude and get the new state
 *  Directly called after a succeslfull gyro+accel reading
 */
static void ins_ekf2_publish_attitude(uint32_t stamp)
{
  imuSample imu_sample = {};
  imu_sample.time_us = stamp;
  imu_sample.delta_ang_dt = ekf2.gyro_dt * 1.e-6f;
  imu_sample.delta_ang = Vector3f{ekf2.gyro.p, ekf2.gyro.q, ekf2.gyro.r} * imu_sample.delta_ang_dt;
  imu_sample.delta_vel_dt = ekf2.accel_dt * 1.e-6f;
  imu_sample.delta_vel = Vector3f{ekf2.accel.x, ekf2.accel.y, ekf2.accel.z} * imu_sample.delta_vel_dt;
  ekf.setIMUData(imu_sample);

  if (ekf.attitude_valid()) {
    // Calculate the quaternion
    struct FloatQuat ltp_to_body_quat;
    const Quatf att_q{ekf.calculate_quaternion()};
    ltp_to_body_quat.qi = att_q(0);
    ltp_to_body_quat.qx = att_q(1);
    ltp_to_body_quat.qy = att_q(2);
    ltp_to_body_quat.qz = att_q(3);

    // Publish it to the state
    stateSetNedToBodyQuat_f(&ltp_to_body_quat);

    /* Check the quaternion reset state */
    float delta_q_reset[4];
    uint8_t quat_reset_counter;
    ekf.get_quat_reset(delta_q_reset, &quat_reset_counter);

#ifndef NO_RESET_UPDATE_SETPOINT_HEADING

    if (ekf2.quat_reset_counter < quat_reset_counter) {
      float psi = matrix::Eulerf(matrix::Quatf(delta_q_reset)).psi();
#if defined STABILIZATION_ATTITUDE_TYPE_INT
      stab_att_sp_euler.psi += ANGLE_BFP_OF_REAL(psi);
#else
      stab_att_sp_euler.psi += psi;
#endif
      guidance_h.sp.heading += psi;
      guidance_h.rc_sp.psi += psi;
      nav_heading += ANGLE_BFP_OF_REAL(psi);
      guidance_h_read_rc(autopilot_in_flight());
      stabilization_attitude_enter();
      ekf2.quat_reset_counter = quat_reset_counter;
    }
#endif

    /* Get in-run gyro bias */
    struct FloatRates body_rates;
    Vector3f gyro_bias{ekf.getGyroBias()};
    body_rates.p = ekf2.gyro.p - gyro_bias(0);
    body_rates.q = ekf2.gyro.q - gyro_bias(1);
    body_rates.r = ekf2.gyro.r - gyro_bias(2);

    // Publish it to the state
    stateSetBodyRates_f(&body_rates);

    /* Get the in-run acceleration bias */
    struct Int32Vect3 accel;
    Vector3f accel_bias{ekf.getAccelBias()};
    accel.x = ACCEL_BFP_OF_REAL(ekf2.accel.x - accel_bias(0));
    accel.y = ACCEL_BFP_OF_REAL(ekf2.accel.y - accel_bias(1));
    accel.z = ACCEL_BFP_OF_REAL(ekf2.accel.z - accel_bias(2));

    // Publish it to the state
    stateSetAccelBody_i(&accel);
  }

  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = true;
}

/* Update INS based on Baro information */
static void baro_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float pressure)
{
  baroSample sample;
  sample.time_us = stamp;

  // Calculate the air density
  float rho = pprz_isa_density_of_pressure(pressure, ekf2.temp);
  ekf.set_air_density(rho);

  // Calculate the height above mean sea level based on pressure
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
  rangeSample sample;
  sample.time_us = stamp;
  sample.rng = distance;
  sample.quality = -1;

  ekf.setRangeData(sample);
}

/* Update INS based on Gyro information */
static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  FloatRates imu_rate;
  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ekf2.body_to_imu);

  // Convert Gyro information to float
  RATES_FLOAT_OF_BFP(imu_rate, *gyro);

  // Rotate with respect to Body To IMU
  float_rmat_transp_ratemult(&ekf2.gyro, body_to_imu_rmat, &imu_rate);

  // Calculate the Gyro interval
  if (ekf2.gyro_stamp > 0) {
    ekf2.gyro_dt = stamp - ekf2.gyro_stamp;
    ekf2.gyro_valid = true;
  }
  ekf2.gyro_stamp = stamp;

  /* When Gyro and accelerometer are valid enter it into the EKF */
  if (ekf2.gyro_valid && ekf2.accel_valid) {
    ins_ekf2_publish_attitude(stamp);
  }
}

/* Update INS based on Accelerometer information */
static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  struct FloatVect3 accel_imu;
  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ekf2.body_to_imu);

  // Convert Accelerometer information to float
  ACCELS_FLOAT_OF_BFP(accel_imu, *accel);

  // Rotate with respect to Body To IMU
  float_rmat_transp_vmult(&ekf2.accel, body_to_imu_rmat, &accel_imu);

  // Calculate the Accelerometer interval
  if (ekf2.accel_stamp > 0) {
    ekf2.accel_dt = stamp - ekf2.accel_stamp;
    ekf2.accel_valid = true;
  }
  ekf2.accel_stamp = stamp;

  /* When Gyro and accelerometer are valid enter it into the EKF */
  if (ekf2.gyro_valid && ekf2.accel_valid) {
    ins_ekf2_publish_attitude(stamp);
  }
}

/* Update INS based on Magnetometer information */
static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t stamp,
                   struct Int32Vect3 *mag)
{
  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ekf2.body_to_imu);
  struct FloatVect3 mag_gauss, mag_body;
  magSample sample;
  sample.time_us = stamp;

  // Convert Magnetometer information to float and to radius 0.2f
  MAGS_FLOAT_OF_BFP(mag_gauss, *mag);
  mag_gauss.x *= 0.4f;
  mag_gauss.y *= 0.4f;
  mag_gauss.z *= 0.4f;

  // Rotate with respect to Body To IMU
  float_rmat_transp_vmult(&mag_body, body_to_imu_rmat, &mag_gauss);

  // Publish information to the EKF
  sample.mag(0) = mag_body.x;
  sample.mag(1) = mag_body.y;
  sample.mag(2) = mag_body.z;

  ekf.setMagData(sample);
  ekf2.got_imu_data = true;
}

/* Update INS based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
  gps_message gps_msg = {};
  gps_msg.time_usec = stamp;
  gps_msg.lat = gps_s->lla_pos.lat;
  gps_msg.lon = gps_s->lla_pos.lon;
  gps_msg.alt = gps_s->hmsl;
#if INS_EKF2_GPS_COURSE_YAW
  gps_msg.yaw = wrap_pi((float)gps_s->course / 1e7);
  gps_msg.yaw_offset = 0;
#else
  gps_msg.yaw = NAN;
  gps_msg.yaw_offset = NAN;
#endif
  gps_msg.fix_type = gps_s->fix;
  gps_msg.eph = gps_s->hacc / 100.0;
  gps_msg.epv = gps_s->vacc / 100.0;
  gps_msg.sacc = gps_s->sacc / 100.0;
  gps_msg.vel_m_s = gps_s->gspeed / 100.0;
  gps_msg.vel_ned(0) = (gps_s->ned_vel.x) / 100.0;
  gps_msg.vel_ned(1) = (gps_s->ned_vel.y) / 100.0;
  gps_msg.vel_ned(2) = (gps_s->ned_vel.z) / 100.0;
  gps_msg.vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  gps_msg.nsats = gps_s->num_sv;
  gps_msg.pdop = gps_s->pdop;

  ekf.setGpsData(gps_msg);
}

/* Save the Body to IMU information */
static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  orientationSetQuat_f(&ekf2.body_to_imu, q_b2i_f);
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
  flowSample sample;
  sample.time_us = stamp;

  // Wait for two measurements in order to integrate
  if (ekf2.flow_stamp <= 0) {
    ekf2.flow_stamp = stamp;
    return;
  }

  // Calculate the timestamp
  sample.dt = (stamp - ekf2.flow_stamp);
  ekf2.flow_stamp = stamp;

  /* Build integrated flow and gyro messages for filter
  NOTE: pure rotations should result in same flow_x and
  gyro_roll and same flow_y and gyro_pitch */
  Vector2f flowdata;
  flowdata(0) = RadOfDeg(flow_y) * (1e-6 *
                                    sample.dt);                       // INTEGRATED FLOW AROUND Y AXIS (RIGHT -X, LEFT +X)
  flowdata(1) = - RadOfDeg(flow_x) * (1e-6 *
                                      sample.dt);                     // INTEGRATED FLOW AROUND X AXIS (FORWARD +Y, BACKWARD -Y)

  sample.quality = quality;                     // quality indicator between 0 and 255
  sample.flow_xy_rad =
    flowdata;                   // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
  sample.gyro_xyz = Vector3f{NAN, NAN, NAN};    // measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive

  // Update the optical flow data based on the callback
  ekf.setOpticalFlowData(sample);
}
