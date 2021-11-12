/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ahrs/ahrs_int_cmpl_quat_wrapper.c
 *
 * Paparazzi specific wrapper to run floating point complementary filter.
 */

#include "modules/ahrs/ahrs_int_cmpl_quat_wrapper.h"
#include "modules/ahrs/ahrs.h"
#include "modules/core/abi.h"
#include "state.h"

#ifndef AHRS_ICQ_OUTPUT_ENABLED
#define AHRS_ICQ_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_ICQ_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool ahrs_icq_output_enabled;
static uint32_t ahrs_icq_last_stamp;
static uint8_t ahrs_icq_id = AHRS_COMP_ID_ICQ;

static void set_body_state_from_quat(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

static void send_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_QUAT_INT(trans, dev, AC_ID,
                              &ahrs_icq.weight,
                              &ahrs_icq.ltp_to_imu_quat.qi,
                              &ahrs_icq.ltp_to_imu_quat.qx,
                              &ahrs_icq.ltp_to_imu_quat.qy,
                              &ahrs_icq.ltp_to_imu_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz),
                              &ahrs_icq_id);
}

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Eulers ltp_to_imu_euler;
  int32_eulers_of_quat(&ltp_to_imu_euler, &ahrs_icq.ltp_to_imu_quat);
  struct Int32Eulers *eulers = stateGetNedToBodyEulers_i();
  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
                               &ltp_to_imu_euler.phi,
                               &ltp_to_imu_euler.theta,
                               &ltp_to_imu_euler.psi,
                               &(eulers->phi),
                               &(eulers->theta),
                               &(eulers->psi),
                               &ahrs_icq_id);
}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &ahrs_icq.gyro_bias.p, &ahrs_icq.gyro_bias.q,
                                   &ahrs_icq.gyro_bias.r, &ahrs_icq_id);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatVect3 h_float;
  h_float.x = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.x);
  h_float.y = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.y);
  h_float.z = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.z);
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &h_float.x, &h_float.y, &h_float.z, &ahrs_icq_id);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_icq.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_icq_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_icq_id, &mde, &val);
}
#endif


/** ABI binding for IMU data.
 * Used for gyro, accel ABI messages.
 */
#ifndef AHRS_ICQ_IMU_ID
#define AHRS_ICQ_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(AHRS_ICQ_IMU_ID)
/** ABI binding for magnetometer data.
 * Used for IMU_MAG_INT32 ABI messages.
 */
#ifndef AHRS_ICQ_MAG_ID
#define AHRS_ICQ_MAG_ID AHRS_ICQ_IMU_ID
#endif
PRINT_CONFIG_VAR(AHRS_ICQ_MAG_ID)
/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef AHRS_ICQ_GPS_ID
#define AHRS_ICQ_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(AHRS_ICQ_GPS_ID)
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;
static abi_event geo_mag_ev;
static abi_event gps_ev;


static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  ahrs_icq_last_stamp = stamp;
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_ICQ propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_ICQ propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_icq.status == AHRS_ICQ_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();
  }
#endif
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t __attribute__((unused)) stamp,
                     struct Int32Vect3 *accel)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS int_cmpl_quat accel update.")
  PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
#endif
}

static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t __attribute__((unused)) stamp,
                   struct Int32Vect3 *mag)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_MAG_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat mag update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_update_mag(mag, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_MAG_CORRECT_FREQUENCY for AHRS int_cmpl_quat mag update.")
  PRINT_CONFIG_VAR(AHRS_MAG_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_MAG_CORRECT_FREQUENCY);
    ahrs_icq_update_mag(mag, dt);
    set_body_state_from_quat();
  }
#endif
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_icq.is_aligned) {
    if (ahrs_icq_align(lp_gyro, lp_accel, lp_mag)) {
      set_body_state_from_quat();
    }
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_icq_set_body_to_imu_quat(q_b2i_f);
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  VECT3_ASSIGN(ahrs_icq.mag_h, MAG_BFP_OF_REAL(h->x), MAG_BFP_OF_REAL(h->y),
               MAG_BFP_OF_REAL(h->z));
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ahrs_icq_update_gps(gps_s);
}

static bool ahrs_icq_enable_output(bool enable)
{
  ahrs_icq_output_enabled = enable;
  return ahrs_icq_output_enabled;
}

/** Rotate angles and rates from imu to body frame and set state */
static void set_body_state_from_quat(void)
{
  if (ahrs_icq_output_enabled) {
    /* Compute LTP to BODY quaternion */
    struct Int32Quat ltp_to_body_quat;
    struct Int32Quat *body_to_imu_quat = orientationGetQuat_i(&ahrs_icq.body_to_imu);
    int32_quat_comp_inv(&ltp_to_body_quat, &ahrs_icq.ltp_to_imu_quat, body_to_imu_quat);
    /* Set state */
    stateSetNedToBodyQuat_i(&ltp_to_body_quat);

    /* compute body rates */
    struct Int32Rates body_rate;
    struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&ahrs_icq.body_to_imu);
    int32_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_icq.imu_rate);
    /* Set state */
    stateSetBodyRates_i(&body_rate);
  }
}

void ahrs_icq_register(void)
{
  ahrs_icq_output_enabled = AHRS_ICQ_OUTPUT_ENABLED;
  ahrs_icq_init();
  ahrs_register_impl(ahrs_icq_enable_output);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(AHRS_ICQ_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_ICQ_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(AHRS_ICQ_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);
  AbiBindMsgGPS(AHRS_ICQ_GPS_ID, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_QUAT_INT, send_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER_INT, send_euler);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_GYRO_BIAS_INT, send_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GEO_MAG, send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
#endif
}
