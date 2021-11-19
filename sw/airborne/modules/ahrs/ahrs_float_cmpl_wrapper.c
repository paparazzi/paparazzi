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
 * @file modules/ahrs/ahrs_float_cmpl_wrapper.c
 *
 * Paparazzi specific wrapper to run floating point complementary filter.
 */

#include "modules/ahrs/ahrs_float_cmpl_wrapper.h"
#include "modules/ahrs/ahrs.h"
#include "modules/core/abi.h"
#include "state.h"

#ifndef AHRS_FC_OUTPUT_ENABLED
#define AHRS_FC_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_FC_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool ahrs_fc_output_enabled;
static uint32_t ahrs_fc_last_stamp;
static uint8_t ahrs_fc_id = AHRS_COMP_ID_FC;

static void compute_body_orientation_and_rates(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers ltp_to_imu_euler;
  float_eulers_of_quat(&ltp_to_imu_euler, &ahrs_fc.ltp_to_imu_quat);
  pprz_msg_send_AHRS_EULER(trans, dev, AC_ID,
                           &ltp_to_imu_euler.phi,
                           &ltp_to_imu_euler.theta,
                           &ltp_to_imu_euler.psi,
                           &ahrs_fc_id);
}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Rates gyro_bias;
  RATES_BFP_OF_REAL(gyro_bias, ahrs_fc.gyro_bias);
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &gyro_bias.p, &gyro_bias.q, &gyro_bias.r, &ahrs_fc_id);
}

static void send_euler_int(struct transport_tx *trans, struct link_device *dev)
{
  /* compute eulers in int (IMU frame) */
  struct FloatEulers ltp_to_imu_euler;
  float_eulers_of_quat(&ltp_to_imu_euler, &ahrs_fc.ltp_to_imu_quat);
  struct Int32Eulers eulers_imu;
  EULERS_BFP_OF_REAL(eulers_imu, ltp_to_imu_euler);

  /* get Eulers in int (body frame) */
  ahrs_fc_recompute_ltp_to_body();
  struct Int32Eulers *eulers_body = orientationGetEulers_i(&ahrs_fc.ltp_to_body);

  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
                               &eulers_imu.phi,
                               &eulers_imu.theta,
                               &eulers_imu.psi,
                               &eulers_body->phi,
                               &eulers_body->theta,
                               &eulers_body->psi,
                               &ahrs_fc_id);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &ahrs_fc.mag_h.x, &ahrs_fc.mag_h.y, &ahrs_fc.mag_h.z, &ahrs_fc_id);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_fc.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_fc_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_fc_id, &mde, &val);
}
#endif


/** ABI binding for IMU data.
 * Used for gyro and accel ABI messages.
 */
#ifndef AHRS_FC_IMU_ID
#define AHRS_FC_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(AHRS_FC_IMU_ID)
/** ABI binding for magnetometer data.
 * Used for IMU_MAG_INT32 ABI messages.
 */
#ifndef AHRS_FC_MAG_ID
#define AHRS_FC_MAG_ID AHRS_FC_IMU_ID
#endif
PRINT_CONFIG_VAR(AHRS_FC_MAG_ID)
/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef AHRS_FC_GPS_ID
#define AHRS_FC_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(AHRS_FC_GPS_ID)
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
  struct FloatRates gyro_f;
  RATES_FLOAT_OF_BFP(gyro_f, *gyro);

#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_FC propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_fc.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_fc_propagate(&gyro_f, dt);
    compute_body_orientation_and_rates();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_FC propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_fc.status == AHRS_FC_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_fc_propagate(&gyro_f, dt);
    compute_body_orientation_and_rates();
  }
#endif
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t __attribute__((unused)) stamp,
                     struct Int32Vect3 *accel)
{
  struct FloatVect3 accel_f;
  ACCELS_FLOAT_OF_BFP(accel_f, *accel);

#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS float_cmpl accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_fc.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_fc_update_accel(&accel_f, dt);
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS float_cmpl accel update.")
  PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_fc.is_aligned) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_fc_update_accel(&accel_f, dt);
  }
#endif
}

static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t __attribute__((unused)) stamp,
                   struct Int32Vect3 *mag)
{
  struct FloatVect3 mag_f;
  MAGS_FLOAT_OF_BFP(mag_f, *mag);

#if USE_AUTO_AHRS_FREQ || !defined(AHRS_MAG_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS float_cmpl mag update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_fc.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_fc_update_mag(&mag_f, dt);
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_MAG_CORRECT_FREQUENCY for AHRS float_cmpl mag update.")
  PRINT_CONFIG_VAR(AHRS_MAG_CORRECT_FREQUENCY)
  if (ahrs_fc.is_aligned) {
    const float dt = 1. / (AHRS_MAG_CORRECT_FREQUENCY);
    ahrs_fc_update_mag(&mag_f, dt);
  }
#endif
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_fc.is_aligned) {
    /* convert to float */
    struct FloatRates gyro_f;
    RATES_FLOAT_OF_BFP(gyro_f, *lp_gyro);
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *lp_accel);
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *lp_mag);
    /* use low passed values to align */
    if (ahrs_fc_align(&gyro_f, &accel_f, &mag_f)) {
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
  ahrs_fc.mag_h = *h;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ahrs_fc_update_gps(gps_s);
  compute_body_orientation_and_rates();
}

static bool ahrs_fc_enable_output(bool enable)
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
    /* recompute LTP to BODY quaternion */
    ahrs_fc_recompute_ltp_to_body();
    struct FloatQuat *ltp_to_body_quat = orientationGetQuat_f(&ahrs_fc.ltp_to_body);
    /* Set state */
    stateSetNedToBodyQuat_f(ltp_to_body_quat);

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
  AbiBindMsgIMU_MAG_INT32(AHRS_FC_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);
  AbiBindMsgGPS(AHRS_FC_GPS_ID, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER, send_euler);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_GYRO_BIAS_INT, send_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER_INT, send_euler_int);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GEO_MAG, send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
#endif
}
