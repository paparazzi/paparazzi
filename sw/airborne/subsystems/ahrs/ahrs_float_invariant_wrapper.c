/*
 * Copyright (C) 2012-2015 Jean-Philippe Condomines, Gautier Hattenberger
 *               2015 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/ahrs/ahrs_float_invariant_wrapper.c
 *
 * Paparazzi specific wrapper to run INVARIANT ahrs filter.
 */

#include "subsystems/ahrs/ahrs_float_invariant_wrapper.h"
#include "subsystems/ahrs.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"
#include "message_pragmas.h"
#include "state.h"

#ifndef AHRS_FINV_OUTPUT_ENABLED
#define AHRS_FINV_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_INV_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool ahrs_finv_output_enabled;
/** last gyro msg timestamp */
static uint32_t ahrs_finv_last_stamp = 0;
static uint8_t ahrs_finv_id = AHRS_COMP_ID_FINV;

static void compute_body_orientation_and_rates(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)
{
  /* compute eulers in int (IMU frame) */
  struct FloatEulers ltp_to_imu_euler;
  float_eulers_of_quat(&ltp_to_imu_euler, &ahrs_float_inv.state.quat);
  struct Int32Eulers eulers_imu;
  EULERS_BFP_OF_REAL(eulers_imu, ltp_to_imu_euler);

  /* compute Eulers in int (body frame) */
  struct FloatQuat ltp_to_body_quat;
  struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ahrs_float_inv.body_to_imu);
  float_quat_comp_inv(&ltp_to_body_quat, &ahrs_float_inv.state.quat, body_to_imu_quat);
  struct FloatEulers ltp_to_body_euler;
  float_eulers_of_quat(&ltp_to_body_euler, &ltp_to_body_quat);
  struct Int32Eulers eulers_body;
  EULERS_BFP_OF_REAL(eulers_body, ltp_to_body_euler);

  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
                               &eulers_imu.phi,
                               &eulers_imu.theta,
                               &eulers_imu.psi,
                               &eulers_body.phi,
                               &eulers_body.theta,
                               &eulers_body.psi,
                               &ahrs_finv_id);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &ahrs_float_inv.mag_h.x,
                        &ahrs_float_inv.mag_h.y,
                        &ahrs_float_inv.mag_h.z, &ahrs_finv_id);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_float_inv.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_finv_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_finv_id, &mde, &val);
}
#endif


/*
 * ABI bindings
 */
/** IMU (gyro, accel) */
#ifndef AHRS_FINV_IMU_ID
#define AHRS_FINV_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(AHRS_FINV_IMU_ID)

/** magnetometer */
#ifndef AHRS_FINV_MAG_ID
#define AHRS_FINV_MAG_ID AHRS_FINV_IMU_ID
#endif
PRINT_CONFIG_VAR(AHRS_FINV_MAG_ID)

static abi_event mag_ev;
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;
static abi_event geo_mag_ev;

/**
 * Call ahrs_float_invariant_propagate on new gyro measurements.
 * Since acceleration measurement is also needed for propagation,
 * use the last stored accel from #ahrs_finv_accel.
 */
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp, struct Int32Rates *gyro)
{
  struct FloatRates gyro_f;
  RATES_FLOAT_OF_BFP(gyro_f, *gyro);

#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS float_invariant propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_float_inv.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_float_invariant_propagate(&gyro_f, dt);
    compute_body_orientation_and_rates();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS float_invariant propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
  if (ahrs_float_inv.is_aligned) {
    ahrs_float_invariant_propagate(&gyro_f, dt);
    compute_body_orientation_and_rates();
  }
#endif

  ahrs_finv_last_stamp = stamp;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  if (ahrs_float_inv.is_aligned) {
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *accel);
    ahrs_float_invariant_update_accel(&accel_f);
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ahrs_float_inv.is_aligned) {
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *mag);
    ahrs_float_invariant_update_mag(&mag_f);
  }
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_float_inv.is_aligned) {
    /* convert to float */
    struct FloatRates gyro_f;
    RATES_FLOAT_OF_BFP(gyro_f, *lp_gyro);
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *lp_accel);
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *lp_mag);
    ahrs_float_invariant_align(&gyro_f, &accel_f, &mag_f);
    compute_body_orientation_and_rates();
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_float_inv_set_body_to_imu_quat(q_b2i_f);
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  ahrs_float_inv.mag_h = *h;
}

static bool ahrs_float_invariant_enable_output(bool enable)
{
  ahrs_finv_output_enabled = enable;
  return ahrs_finv_output_enabled;
}

/**
 * Compute body orientation and rates from imu orientation and rates
 */
static void compute_body_orientation_and_rates(void)
{
  if (ahrs_finv_output_enabled) {
    /* Compute LTP to BODY quaternion */
    struct FloatQuat ltp_to_body_quat;
    struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ahrs_float_inv.body_to_imu);
    float_quat_comp_inv(&ltp_to_body_quat, &ahrs_float_inv.state.quat, body_to_imu_quat);
    /* Set state */
    stateSetNedToBodyQuat_f(&ltp_to_body_quat);

    /* compute body rates */
    struct FloatRates body_rate;
    RATES_DIFF(body_rate, ahrs_float_inv.cmd.rates, ahrs_float_inv.state.bias);
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ahrs_float_inv.body_to_imu);
    float_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &body_rate);
    stateSetBodyRates_f(&body_rate);

  }
}


void ahrs_float_invariant_register(void)
{
  ahrs_finv_output_enabled = AHRS_FINV_OUTPUT_ENABLED;
  ahrs_float_invariant_init();
  ahrs_register_impl(ahrs_float_invariant_enable_output);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_MAG_INT32(AHRS_FINV_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_GYRO_INT32(AHRS_FINV_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_FINV_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_LOWPASSED(AHRS_FINV_IMU_ID, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(AHRS_FINV_IMU_ID, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER_INT, send_att);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GEO_MAG, send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
#endif
}
