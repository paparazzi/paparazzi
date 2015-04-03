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
 * @file subsystems/ahrs/ahrs_float_cmpl_wrapper.c
 *
 * Paparazzi specific wrapper to run floating point complementary filter.
 */

#include "subsystems/ahrs/ahrs_float_cmpl_wrapper.h"
#include "subsystems/ahrs.h"
#include "subsystems/abi.h"
#include "state.h"

#ifndef AHRS_FC_OUTPUT_ENABLED
#define AHRS_FC_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_FC_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool_t ahrs_fc_output_enabled;
static uint32_t ahrs_fc_last_stamp;

static void compute_body_orientation_and_rates(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers ltp_to_imu_euler;
  float_eulers_of_quat(&ltp_to_imu_euler, &ahrs_fc.ltp_to_imu_quat);
  struct Int32Eulers euler_i;
  EULERS_BFP_OF_REAL(euler_i, ltp_to_imu_euler);
  struct Int32Eulers *eulers_body = stateGetNedToBodyEulers_i();
  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
                               &euler_i.phi,
                               &euler_i.theta,
                               &euler_i.psi,
                               &(eulers_body->phi),
                               &(eulers_body->theta),
                               &(eulers_body->psi));
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &ahrs_fc.mag_h.x, &ahrs_fc.mag_h.y, &ahrs_fc.mag_h.z);
}

#ifndef AHRS_FC_FILTER_ID
#define AHRS_FC_FILTER_ID 5
#endif

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t id = AHRS_FC_FILTER_ID;
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_fc.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_fc_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &id, &mde, &val);
}
#endif


/** ABI binding for IMU data.
 * Used for gyro, accel and mag ABI messages.
 */
#ifndef AHRS_FC_IMU_ID
#define AHRS_FC_IMU_ID ABI_BROADCAST
#endif
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
  ahrs_fc_last_stamp = stamp;
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_FC propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_fc.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_fc_propagate(gyro, dt);
    compute_body_orientation_and_rates();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_FC propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_fc.status == AHRS_FC_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_fc_propagate(gyro, dt);
    compute_body_orientation_and_rates();
  }
#endif
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t __attribute__((unused)) stamp,
                     struct Int32Vect3 *accel)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS float_cmpl accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_fc.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_fc_update_accel((struct Int32Vect3 *)accel, dt);
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS float_cmpl accel update.")
  PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_fc.is_aligned) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_fc_update_accel((struct Int32Vect3 *)accel, dt);
  }
#endif
}

static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t __attribute__((unused)) stamp,
                   struct Int32Vect3 *mag)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_MAG_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS float_cmpl mag update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_fc.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_fc_update_mag(mag, dt);
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_MAG_CORRECT_FREQUENCY for AHRS float_cmpl mag update.")
  PRINT_CONFIG_VAR(AHRS_MAG_CORRECT_FREQUENCY)
  if (ahrs_fc.is_aligned) {
    const float dt = 1. / (AHRS_MAG_CORRECT_FREQUENCY);
    ahrs_fc_update_mag(mag, dt);
  }
#endif
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_fc.is_aligned) {
    if (ahrs_fc_align(lp_gyro, lp_accel, lp_mag)) {
      compute_body_orientation_and_rates();
    }
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_fc_set_body_to_imu_quat(q_b2i_f);
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  memcpy(&ahrs_fc.mag_h, h, sizeof(struct FloatVect3));
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ahrs_fc_update_gps(gps_s);
  compute_body_orientation_and_rates();
}

static bool_t ahrs_fc_enable_output(bool_t enable)
{
  ahrs_fc_output_enabled = enable;
  return ahrs_fc_output_enabled;
}

/**
 * Compute body orientation and rates from imu orientation and rates
 */
static void compute_body_orientation_and_rates(void)
{
  if (ahrs_fc_output_enabled) {
    /* Compute LTP to BODY quaternion */
    struct FloatQuat ltp_to_body_quat;
    struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ahrs_fc.body_to_imu);
    float_quat_comp_inv(&ltp_to_body_quat, &ahrs_fc.ltp_to_imu_quat, body_to_imu_quat);
    /* Set state */
    stateSetNedToBodyQuat_f(&ltp_to_body_quat);

    /* compute body rates */
    struct FloatRates body_rate;
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ahrs_fc.body_to_imu);
    float_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_fc.imu_rate);
    stateSetBodyRates_f(&body_rate);
  }
}

void ahrs_fc_register(void)
{
  ahrs_fc_output_enabled = AHRS_FC_OUTPUT_ENABLED;
  ahrs_fc_init();
  ahrs_register_impl(ahrs_fc_enable_output);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(AHRS_FC_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_FC_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(AHRS_FC_IMU_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "AHRS_EULER_INT", send_att);
  register_periodic_telemetry(DefaultPeriodic, "GEO_MAG", send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, "STATE_FILTER_STATUS", send_filter_status);
#endif
}
