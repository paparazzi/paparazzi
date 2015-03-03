/*
 * Copyright (C) 2012-2013 Jean-Philippe Condomines, Gautier Hattenberger
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
 * @file subsystems/ins/ins_float_invariant_wrapper.c
 *
 * Paparazzi specific wrapper to run INVARIANT filter.
 */

#include "subsystems/ins/ins_float_invariant_wrapper.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "message_pragmas.h"

#if PERIODIC_TELEMETRY && !INS_FINV_USE_UTM
#include "subsystems/datalink/telemetry.h"
#include "state.h"
static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  float foo = 0.;
  if (state.ned_initialized_i) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &state.ned_origin_i.ecef.x, &state.ned_origin_i.ecef.y,
                          &state.ned_origin_i.ecef.z, &state.ned_origin_i.lla.lat,
                          &state.ned_origin_i.lla.lon, &state.ned_origin_i.lla.alt,
                          &state.ned_origin_i.hmsl, &foo);
  }
}
#endif


/*
 * ABI bindings
 */
/** baro */
#ifndef INS_FINV_BARO_ID
#if USE_BARO_BOARD
#define INS_FINV_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_FINV_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_FINV_BARO_ID)

/** IMU (gyro, accel) */
#ifndef INS_FINV_IMU_ID
#define INS_FINV_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FINV_IMU_ID)

/** magnetometer */
#ifndef INS_FINV_MAG_ID
#define INS_FINV_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FINV_MAG_ID)

static abi_event baro_ev;
static abi_event mag_ev;
static abi_event gyro_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;
static abi_event geo_mag_ev;

static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{
  ins_float_invariant_update_baro(pressure);
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp, struct Int32Rates *gyro)
{
  PRINT_CONFIG_MSG("Calculating dt for INS float_invariant propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_float_invariant_propagate(gyro, &imu.accel, dt);
  }
  last_stamp = stamp;
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ins_float_inv.is_aligned) {
    ins_float_invariant_update_mag(mag);
  }
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ins_float_inv.is_aligned) {
    ins_float_invariant_align(lp_gyro, lp_accel, lp_mag);
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ins_float_inv_set_body_to_imu_quat(q_b2i_f);
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  memcpy(&ins_float_inv.mag_h, h, sizeof(struct FloatVect3));
}


void ins_float_inv_register(void)
{
  ins_register_impl(ins_float_inv_init, ins_float_inv_update_gps);

 // Bind to ABI messages
  AbiBindMsgBARO_ABS(INS_FINV_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgIMU_MAG_INT32(INS_FINV_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_GYRO_INT32(INS_FINV_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_LOWPASSED(INS_FINV_IMU_ID, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(INS_FINV_IMU_ID, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);

#if PERIODIC_TELEMETRY && !INS_FINV_USE_UTM
  register_periodic_telemetry(DefaultPeriodic, "INS_REF", send_ins_ref);
#endif
}
