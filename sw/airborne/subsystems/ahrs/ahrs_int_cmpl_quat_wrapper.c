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
 * @file subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.c
 *
 * Paparazzi specific wrapper to run floating point complementary filter.
 */

#include "subsystems/ahrs/ahrs_int_cmpl_quat_wrapper.h"
#include "subsystems/ahrs.h"
#include "subsystems/abi.h"
#include "state.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
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
                              &(quat->qz));
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
                               &(eulers->psi));
}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &ahrs_icq.gyro_bias.p, &ahrs_icq.gyro_bias.q, &ahrs_icq.gyro_bias.r);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatVect3 h_float;
  h_float.x = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.x);
  h_float.y = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.y);
  h_float.z = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.z);
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &h_float.x, &h_float.y, &h_float.z);
}
#endif


/** ABI binding for IMU data.
 * Used for gyro, accel and mag ABI messages.
 */
#ifndef AHRS_ICQ_IMU_ID
#define AHRS_ICQ_IMU_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;


static void gyro_cb(uint8_t __attribute__((unused)) sender_id, const uint32_t *stamp,
                    const struct Int32Rates *gyro)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_ICQ propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(*stamp - last_stamp) * 1e-6;
    ahrs_icq_propagate((struct Int32Rates *)gyro, dt);
  }
  last_stamp = *stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_ICQ propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_icq.status == AHRS_ICQ_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_icq_propagate((struct Int32Rates *)gyro, dt);
  }
#endif
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id, const uint32_t *stamp,
                     const struct Int32Vect3 *accel)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(*stamp - last_stamp) * 1e-6;
    ahrs_icq_update_accel((struct Int32Vect3 *)accel, dt);
  }
  last_stamp = *stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS int_cmpl_quat accel update.")
  PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_icq_update_accel((struct Int32Vect3 *)accel, dt);
  }
#endif
}

static void mag_cb(uint8_t __attribute__((unused)) sender_id, const uint32_t *stamp,
                   const struct Int32Vect3 *mag)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_MAG_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat mag update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(*stamp - last_stamp) * 1e-6;
    ahrs_icq_update_mag((struct Int32Vect3 *)mag, dt);
  }
  last_stamp = *stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_MAG_CORRECT_FREQUENCY for AHRS int_cmpl_quat mag update.")
  PRINT_CONFIG_VAR(AHRS_MAG_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_MAG_CORRECT_FREQUENCY);
    ahrs_icq_update_mag((struct Int32Vect3 *)mag, dt);
  }
#endif
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       const uint32_t *stamp __attribute__((unused)),
                       const struct Int32Rates *lp_gyro, const struct Int32Vect3 *lp_accel,
                       const struct Int32Vect3 *lp_mag)
{
  if (!ahrs_icq.is_aligned) {
    ahrs_icq_align((struct Int32Rates *)lp_gyro, (struct Int32Vect3 *)lp_accel,
                   (struct Int32Vect3 *)lp_mag);
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           const struct FloatQuat *q_b2i_f)
{
  ahrs_icq_set_body_to_imu_quat((struct FloatQuat *)q_b2i_f);
}

void ahrs_icq_register(void)
{
  ahrs_register_impl(ahrs_icq_init, ahrs_icq_update_gps);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(AHRS_ICQ_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_ICQ_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(AHRS_ICQ_IMU_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "AHRS_QUAT_INT", send_quat);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_EULER_INT", send_euler);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_GYRO_BIAS_INT", send_bias);
  register_periodic_telemetry(DefaultPeriodic, "GEO_MAG", send_geo_mag);
#endif
}
