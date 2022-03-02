/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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
#include "EKF/ekf.h"
#include "math/pprz_isa.h"
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

/** default AGL sensor to use in INS */
#ifndef INS_EKF2_AGL_ID
#define INS_EKF2_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_AGL_ID)

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

/* Flow sensor X offset from IMU position in meters */
#ifndef INS_EKF2_FLOW_OFFSET_X
#define INS_EKF2_FLOW_OFFSET_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_OFFSET_X)

/* Flow sensor Y offset from IMU position in meters */
#ifndef INS_EKF2_FLOW_OFFSET_Y
#define INS_EKF2_FLOW_OFFSET_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_OFFSET_Y)

/* Flow sensor Z offset from IMU position in meters */
#ifndef INS_EKF2_FLOW_OFFSET_Z
#define INS_EKF2_FLOW_OFFSET_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_OFFSET_Z)

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

/* All registered ABI events */
static abi_event agl_ev;
static abi_event baro_ev;
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event gps_ev;
static abi_event body_to_imu_ev;
static abi_event optical_flow_ev;

/* Build optical flow and gps message struct based on flow message defined in common.h */
struct gps_message gps_msg = {};
struct flow_message flow_msg = {};

/* All ABI callbacks */
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static void optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x, int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);

/* Main EKF2 structure for keeping track of the status and use cross messaging */
struct ekf2_t
{

  // stamp and dt for sensors
  uint32_t gyro_stamp;
  uint32_t gyro_dt;
  uint32_t accel_stamp;
  uint32_t accel_dt;
  uint32_t flow_stamp;
  uint32_t flow_dt;

  // gyro and accellerometer values
  FloatRates gyro;
  FloatVect3 accel;
  bool gyro_valid;
  bool accel_valid;

  // optical flow and gyro values
  float flow_quality;
  float flow_x;
  float flow_y;
  float gyro_roll;
  float gyro_pitch;
  float gyro_yaw;
  float offset_x;
  float offset_y;
  float offset_z;

  // optical flow takeover
  float flow_innov;

  uint8_t quat_reset_counter;
  uint64_t ltp_stamp;
  struct LtpDef_i ltp_def;
  struct OrientationReps body_to_imu;
  bool got_imu_data;
};

/* Static local functions */
static void ins_ekf2_publish_attitude(uint32_t stamp);

/* Static local variables */
static Ekf ekf;                                   ///< EKF class itself
static parameters *ekf_params;                    ///< The EKF parameters
struct ekf2_t ekf2;                               ///< Local EKF2 status structure
static uint8_t ahrs_ekf2_id = AHRS_COMP_ID_EKF2;  ///< Component ID for EKF

/* External paramters */
struct ekf2_parameters_t ekf2_params;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

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
  uint16_t gps_check_status, filter_fault_status, soln_status;
  uint32_t control_mode;
  ekf.get_gps_check_status(&gps_check_status);
  ekf.get_filter_fault_status(&filter_fault_status);
  ekf.get_control_mode(&control_mode);
  ekf.get_ekf_soln_status(&soln_status);

  uint16_t innov_test_status;
  float mag, vel, pos, hgt, tas, hagl, flow, beta, mag_decl;
  uint8_t terrain_valid, dead_reckoning;
  ekf.get_innovation_test_status(&innov_test_status, &mag, &vel, &pos, &hgt, &tas, &hagl, &beta);
  ekf.get_flow_innov(&flow);
  ekf.get_mag_decl_deg(&mag_decl);

  uint32_t fix_status = (control_mode >> 2) & 1;

  if (ekf.get_terrain_valid()) {
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
                         &fix_status, &filter_fault_status, &gps_check_status, &soln_status,
                         &innov_test_status, &mag, &vel, &pos, &hgt, &tas, &hagl, &flow, &beta,
                         &mag_decl, &terrain_valid, &dead_reckoning);
}

static void send_ins_ekf2_ext(struct transport_tx *trans, struct link_device *dev)
{
  float gps_drift[3], vibe[3];
  bool gps_blocked;
  uint8_t gps_blocked_b;
  ekf.get_gps_drift_metrics(gps_drift, &gps_blocked);
  ekf.get_imu_vibe_metrics(vibe);
  gps_blocked_b = gps_blocked;

  pprz_msg_send_INS_EKF2_EXT(trans, dev, AC_ID,
                             &gps_drift[0], &gps_drift[1], &gps_drift[2], &gps_blocked_b,
                             &vibe[0], &vibe[1], &vibe[2]);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t control_mode;
  uint16_t filter_fault_status;
  uint8_t mde = 0;
  ekf.get_control_mode(&control_mode);
  ekf.get_filter_fault_status(&filter_fault_status);

  // Check the alignment and if GPS is fused
  if ((control_mode & 0x7) == 0x7) {
    mde = 3;
  } else if ((control_mode & 0x7) == 0x3) {
    mde = 4;
  } else {
    mde = 2;
  }

  // Check if there is a covariance error
  if (filter_fault_status) {
    mde = 6;
  }

  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_ekf2_id, &mde, &filter_fault_status);
}

static void send_wind_info_ret(struct transport_tx *trans, struct link_device *dev)
{
  float velNE_wind[2], tas;
  uint8_t flags = 0x5;
  float f_zero = 0;

  ekf.get_wind_velocity(velNE_wind);
  ekf.get_true_airspeed(&tas);

  pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID, &flags, &velNE_wind[1], &velNE_wind[0], &f_zero, &tas);
}

static void send_ahrs_bias(struct transport_tx *trans, struct link_device *dev)
{
  float accel_bias[3], gyro_bias[3], states[24];
  ekf.get_accel_bias(accel_bias);
  ekf.get_gyro_bias(gyro_bias);
  ekf.get_state_delayed(states);

  pprz_msg_send_AHRS_BIAS(trans, dev, AC_ID, &accel_bias[0], &accel_bias[1], &accel_bias[2],
                          &gyro_bias[0], &gyro_bias[1], &gyro_bias[2], &states[19], &states[20], &states[21]);
}
#endif

/* Initialize the EKF */
void ins_ekf2_init(void)
{
  /* Get the ekf parameters */
  ekf_params = ekf.getParamHandle();
  ekf_params->mag_fusion_type = MAG_FUSE_TYPE_HEADING;
  ekf_params->is_moving_scaler = 0.8f;
  ekf_params->fusion_mode = INS_EKF2_FUSION_MODE;
  ekf_params->vdist_sensor_type = INS_EKF2_VDIST_SENSOR_TYPE;
  ekf_params->gps_check_mask = INS_EKF2_GPS_CHECK_MASK;

  /* Set optical flow parameters */
  ekf_params->flow_qual_min = INS_EKF2_MIN_FLOW_QUALITY;
  ekf_params->flow_delay_ms = INS_EKF2_FLOW_SENSOR_DELAY;
  ekf_params->range_delay_ms = INS_EKF2_FLOW_SENSOR_DELAY;
  ekf_params->flow_noise = INS_EKF2_FLOW_NOISE;
	ekf_params->flow_noise_qual_min = INS_EKF2_FLOW_NOISE_QMIN;
	ekf_params->flow_innov_gate = INS_EKF2_FLOW_INNOV_GATE;

  /* Set flow sensor offset from IMU position in xyz (m) */
  ekf2.offset_x = INS_EKF2_FLOW_OFFSET_X;
  ekf2.offset_y = INS_EKF2_FLOW_OFFSET_Y;
  ekf2.offset_z = INS_EKF2_FLOW_OFFSET_Z;
  ekf_params->flow_pos_body = {0.001f*ekf2.offset_x, 0.001f*ekf2.offset_y, 0.001f*ekf2.offset_z};

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

  /* Initialize the range sensor limits */
  ekf.set_rangefinder_limits(INS_EKF2_SONAR_MIN_RANGE, INS_EKF2_SONAR_MAX_RANGE);

  /* Initialize the flow sensor limits */
  ekf.set_optical_flow_limits(INS_EKF2_MAX_FLOW_RATE, INS_EKF2_SONAR_MIN_RANGE, INS_EKF2_SONAR_MAX_RANGE);

#if PERIODIC_TELEMETRY
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
  AbiBindMsgAGL(INS_EKF2_AGL_ID, &agl_ev, agl_cb);
  AbiBindMsgIMU_GYRO_INT32(INS_EKF2_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(INS_EKF2_ACCEL_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(INS_EKF2_MAG_ID, &mag_ev, mag_cb);
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

    // Get the status from the EKF
    filter_control_status_u control_status;
    ekf.get_control_mode(&control_status.value);

    // Only publish position after successful alignment
    if (control_status.flags.tilt_align) {
      /* Get the position */
      float pos_f[3] = {};
      struct NedCoor_f pos;
      ekf.get_position(pos_f);
      pos.x = pos_f[0];
      pos.y = pos_f[1];
      pos.z = pos_f[2];

      // Publish to the state
      stateSetPositionNed_f(&pos);

      /* Get the velocity in NED frame */
      float vel_f[3] = {};
      struct NedCoor_f speed;
      ekf.get_velocity(vel_f);
      speed.x = vel_f[0];
      speed.y = vel_f[1];
      speed.z = vel_f[2];

      // Publish to state
      stateSetSpeedNed_f(&speed);

      /* Get the accelrations in NED frame */
      float vel_deriv_f[3] = {};
      struct NedCoor_f accel;
      ekf.get_vel_deriv_ned(vel_deriv_f);
      accel.x = vel_deriv_f[0];
      accel.y = vel_deriv_f[1];
      accel.z = vel_deriv_f[2];

      // Publish to state
      stateSetAccelNed_f(&accel);

      /* Get local origin */
      // Position of local NED origin in GPS / WGS84 frame
      struct map_projection_reference_s ekf_origin = {};
      float ref_alt;
      struct LlaCoor_i lla_ref;
      uint64_t origin_time;

      // Only update the origin when the state estimator has updated the origin
      bool ekf_origin_valid = ekf.get_ekf_origin(&origin_time, &ekf_origin, &ref_alt);
      if (ekf_origin_valid && (origin_time > ekf2.ltp_stamp)) {
        lla_ref.lat = ekf_origin.lat_rad * 180.0 / M_PI * 1e7; // Reference point latitude in degrees
        lla_ref.lon = ekf_origin.lon_rad * 180.0 / M_PI * 1e7; // Reference point longitude in degrees
        lla_ref.alt = ref_alt * 1000.0;
        ltp_def_from_lla_i(&ekf2.ltp_def, &lla_ref);
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
  ekf_params->mag_fusion_type = ekf2_params.mag_fusion_type = unk;
}

void ins_ekf2_remove_gps(int32_t mode)
{
  if (mode) {
    ekf_params->fusion_mode = ekf2_params.fusion_mode = (MASK_USE_OF | MASK_USE_GPSYAW);
  } else {
    ekf_params->fusion_mode = ekf2_params.fusion_mode = INS_EKF2_FUSION_MODE;
  }
}

/** Publish the attitude and get the new state
 *  Directly called after a succeslfull gyro+accel reading
 */
static void ins_ekf2_publish_attitude(uint32_t stamp)
{
  imuSample imu_sample;
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
    float gyro_bias[3];
    ekf.get_gyro_bias(gyro_bias);
    body_rates.p = ekf2.gyro.p - gyro_bias[0];
    body_rates.q = ekf2.gyro.q - gyro_bias[1];
    body_rates.r = ekf2.gyro.r - gyro_bias[2];

    // Publish it to the state
    stateSetBodyRates_f(&body_rates);

    /* Get the in-run acceleration bias */
    struct Int32Vect3 accel;
    float accel_bias[3];
    ekf.get_accel_bias(accel_bias);
    accel.x = ACCEL_BFP_OF_REAL(ekf2.accel.x - accel_bias[0]);
    accel.y = ACCEL_BFP_OF_REAL(ekf2.accel.y - accel_bias[1]);
    accel.z = ACCEL_BFP_OF_REAL(ekf2.accel.z - accel_bias[2]);

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
  // Calculate the air density
  float rho = pprz_isa_density_of_pressure(pressure,
                                           20.0f); // TODO: add temperature compensation now set to 20 degree celcius
  ekf.set_air_density(rho);

  // Calculate the height above mean sea level based on pressure
  float height_amsl_m = pprz_isa_height_of_pressure_full(pressure,
                                                         101325.0); //101325.0 defined as PPRZ_ISA_SEA_LEVEL_PRESSURE in pprz_isa.h
  ekf.setBaroData(stamp, height_amsl_m);
}

/* Update INS based on AGL information */
static void agl_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float distance)
{
  ekf.setRangeData(stamp, distance);
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

  // Convert Magnetometer information to float and to radius 0.2f
  MAGS_FLOAT_OF_BFP(mag_gauss, *mag);
  mag_gauss.x *= 0.4f;
  mag_gauss.y *= 0.4f;
  mag_gauss.z *= 0.4f;

  // Rotate with respect to Body To IMU
  float_rmat_transp_vmult(&mag_body, body_to_imu_rmat, &mag_gauss);

  // Publish information to the EKF
  float mag_r[3];
  mag_r[0] = mag_body.x;
  mag_r[1] = mag_body.y;
  mag_r[2] = mag_body.z;

  ekf.setMagData(stamp, mag_r);
  ekf2.got_imu_data = true;
}

/* Update INS based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
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
  gps_msg.vel_ned[0] = (gps_s->ned_vel.x) / 100.0;
  gps_msg.vel_ned[1] = (gps_s->ned_vel.y) / 100.0;
  gps_msg.vel_ned[2] = (gps_s->ned_vel.z) / 100.0;
  gps_msg.vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  gps_msg.nsats = gps_s->num_sv;
  gps_msg.gdop = 0.0f;

  ekf.setGpsData(stamp, gps_msg);
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
  // update time
  ekf2.flow_dt = stamp - ekf2.flow_stamp;
  ekf2.flow_stamp = stamp;

  /* Build integrated flow and gyro messages for filter
  NOTE: pure rotations should result in same flow_x and 
  gyro_roll and same flow_y and gyro_pitch */
  ekf2.flow_quality = quality;
  ekf2.flow_x = RadOfDeg(flow_y) * (1e-6 * ekf2.flow_dt);                       // INTEGRATED FLOW AROUND Y AXIS (RIGHT -X, LEFT +X)
  ekf2.flow_y = - RadOfDeg(flow_x) * (1e-6 * ekf2.flow_dt);                     // INTEGRATED FLOW AROUND X AXIS (FORWARD +Y, BACKWARD -Y)
  ekf2.gyro_roll = NAN;
  ekf2.gyro_pitch = NAN;
  ekf2.gyro_yaw = NAN;

  /* once callback initiated, build the 
  optical flow message with what is received */
  flow_msg.quality = quality;                                                   // quality indicator between 0 and 255
  flow_msg.flowdata = Vector2f(ekf2.flow_x, ekf2.flow_y);                       // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
  flow_msg.gyrodata = Vector3f{ekf2.gyro_roll, ekf2.gyro_pitch, ekf2.gyro_yaw}; // measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive
  flow_msg.dt = ekf2.flow_dt;                                                   // amount of integration time (usec)

  // update the optical flow data based on the callback
  ekf.setOpticalFlowData(stamp, &flow_msg);
}
