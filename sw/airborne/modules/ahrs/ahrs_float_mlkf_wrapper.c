/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/ahrs/ahrs_float_mlkf_wrapper.c
 *
 * Paparazzi specific wrapper to run MLKF filter.
 */

#include "modules/ahrs/ahrs_float_mlkf_wrapper.h"
#include "modules/ahrs/ahrs.h"
#include "modules/core/abi.h"
#include "state.h"

#ifndef AHRS_MLKF_OUTPUT_ENABLED
#define AHRS_MLKF_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_MLKF_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool ahrs_mlkf_output_enabled;
static uint32_t ahrs_mlkf_last_stamp;
static uint8_t ahrs_mlkf_id = AHRS_COMP_ID_MLKF;

static void set_body_state_from_quat(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers ltp_to_imu_euler;
  float_eulers_of_quat(&ltp_to_imu_euler, &ahrs_mlkf.ltp_to_imu_quat);
  pprz_msg_send_AHRS_EULER(trans, dev, AC_ID,
                           &ltp_to_imu_euler.phi,
                           &ltp_to_imu_euler.theta,
                           &ltp_to_imu_euler.psi,
                           &ahrs_mlkf_id);
}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Rates gyro_bias;
  RATES_BFP_OF_REAL(gyro_bias, ahrs_mlkf.gyro_bias);
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &gyro_bias.p, &gyro_bias.q, &gyro_bias.r, &ahrs_mlkf_id);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &ahrs_mlkf.mag_h.x, &ahrs_mlkf.mag_h.y, &ahrs_mlkf.mag_h.z, &ahrs_mlkf_id);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_mlkf.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_mlkf_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_mlkf_id, &mde, &val);
}
#endif


/** ABI binding for IMU data.
 * Used for gyro, accel ABI messages.
 */
#ifndef AHRS_MLKF_IMU_ID
#define AHRS_MLKF_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(AHRS_MLKF_IMU_ID)
/** ABI binding for magnetometer data.
 * Used for IMU_MAG_INT32 ABI messages.
 */
#ifndef AHRS_MLKF_MAG_ID
#define AHRS_MLKF_MAG_ID AHRS_MLKF_IMU_ID
#endif
PRINT_CONFIG_VAR(AHRS_MLKF_MAG_ID)
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;
static abi_event geo_mag_ev;


static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  ahrs_mlkf_last_stamp = stamp;
  struct FloatRates gyro_f;
  RATES_FLOAT_OF_BFP(gyro_f, *gyro);

#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_MLKF propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_mlkf.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_mlkf_propagate(&gyro_f, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_MLKF propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_mlkf.status == AHRS_MLKF_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_mlkf_propagate(&gyro_f, dt);
    set_body_state_from_quat();
  }
#endif
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  if (ahrs_mlkf.is_aligned) {
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *accel);
    ahrs_mlkf_update_accel(&accel_f);
    set_body_state_from_quat();
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ahrs_mlkf.is_aligned) {
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *mag);
    ahrs_mlkf_update_mag(&mag_f);
    set_body_state_from_quat();
  }
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_mlkf.is_aligned) {
    /* convert to float */
    struct FloatRates gyro_f;
    RATES_FLOAT_OF_BFP(gyro_f, *lp_gyro);
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *lp_accel);
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *lp_mag);
    /* set initial body orientation in state interface if alignment was successful */
    if (ahrs_mlkf_align(&gyro_f, &accel_f, &mag_f)) {
      set_body_state_from_quat();
    }
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_mlkf_set_body_to_imu_quat(q_b2i_f);
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  ahrs_mlkf.mag_h = *h;
}

static bool ahrs_mlkf_enable_output(bool enable)
{
  ahrs_mlkf_output_enabled = enable;
  return ahrs_mlkf_output_enabled;
}

/**
 * Compute body orientation and rates from imu orientation and rates
 */
static void set_body_state_from_quat(void)
{
  if (ahrs_mlkf_output_enabled) {
    struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ahrs_mlkf.body_to_imu);
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ahrs_mlkf.body_to_imu);

    /* Compute LTP to BODY quaternion */
    struct FloatQuat ltp_to_body_quat;
    float_quat_comp_inv(&ltp_to_body_quat, &ahrs_mlkf.ltp_to_imu_quat, body_to_imu_quat);
    /* Set in state interface */
    stateSetNedToBodyQuat_f(&ltp_to_body_quat);

    /* compute body rates */
    struct FloatRates body_rate;
    float_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_mlkf.imu_rate);
    /* Set state */
    stateSetBodyRates_f(&body_rate);
  }
}

void ahrs_mlkf_register(void)
{
  ahrs_mlkf_output_enabled = AHRS_MLKF_OUTPUT_ENABLED;
  ahrs_mlkf_init();
  ahrs_register_impl(ahrs_mlkf_enable_output);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO(AHRS_MLKF_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(AHRS_MLKF_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG(AHRS_MLKF_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER, send_euler);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_GYRO_BIAS_INT, send_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GEO_MAG, send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
#endif
}

