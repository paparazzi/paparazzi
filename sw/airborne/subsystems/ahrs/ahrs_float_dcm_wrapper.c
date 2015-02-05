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
 * @file subsystems/ahrs/ahrs_float_dcm_wrapper.c
 *
 * Paparazzi specific wrapper to run floating point complementary filter.
 */

#include "subsystems/ahrs/ahrs_float_dcm_wrapper.h"
#include "subsystems/ahrs.h"
#include "subsystems/abi.h"
#include "state.h"

/** ABI binding for IMU data.
 * Used for gyro, accel and mag ABI messages.
 */
#ifndef AHRS_DCM_IMU_ID
#define AHRS_DCM_IMU_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;


static void gyro_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp,
                    struct Int32Rates *gyro)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS dcm propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_dcm.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_dcm_propagate(gyro, dt);
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS dcm propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_dcm.is_aligned) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_dcm_propagate(gyro, dt);
  }
#endif
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  if (ahrs_dcm.is_aligned) {
    ahrs_dcm_update_accel(accel);
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ahrs_dcm.is_aligned) {
    ahrs_dcm_update_mag(mag);
  }
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_dcm.is_aligned) {
    ahrs_dcm_align(lp_gyro, lp_accel, lp_mag);
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_dcm_set_body_to_imu_quat(q_b2i_f);
}

void ahrs_dcm_register(void)
{
  ahrs_register_impl(ahrs_dcm_init, ahrs_dcm_update_gps);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(AHRS_DCM_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_DCM_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(AHRS_DCM_IMU_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
}
