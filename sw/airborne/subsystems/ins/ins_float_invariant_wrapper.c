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
#include "mcu_periph/sys_time.h"
#include "message_pragmas.h"
#if USE_AHRS_ALIGNER
#include "subsystems/ahrs/ahrs_aligner.h"
#endif

#ifndef INS_FINV_FILTER_ID
#define INS_FINV_FILTER_ID 2
#endif

/** last accel measurement */
static struct FloatVect3 ins_finv_accel;

/** last gyro msg timestamp */
static uint32_t ins_finv_last_stamp = 0;

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

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t id = INS_FINV_FILTER_ID;
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ins_float_inv.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ins_finv_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &id, &mde, &val);
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
#if USE_MAGNETOMETER
#ifndef INS_FINV_MAG_ID
#define INS_FINV_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FINV_MAG_ID)
#else
PRINT_CONFIG_MSG("INS invariant use GPS heading as magnetometer")
#endif

/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_FINV_GPS_ID
#define INS_FINV_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_FINV_GPS_ID)

static abi_event baro_ev;
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;
#if USE_MAGNETOMETER
static abi_event mag_ev;
static abi_event geo_mag_ev;
#endif
static abi_event gps_ev;

static void baro_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float pressure)
{
  ins_float_invariant_update_baro(pressure);
}

/**
 * Call ins_float_invariant_propagate on new gyro measurements.
 * Since acceleration measurement is also needed for propagation,
 * use the last stored accel from #ins_finv_accel.
 */
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp, struct Int32Rates *gyro)
{
  struct FloatRates gyro_f;
  RATES_FLOAT_OF_BFP(gyro_f, *gyro);

#if USE_AUTO_INS_FREQ || !defined(INS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for INS float_invariant propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_float_invariant_propagate(&gyro_f, &ins_finv_accel, dt);
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed INS_PROPAGATE_FREQUENCY for INS float_invariant propagation.")
  PRINT_CONFIG_VAR(INS_PROPAGATE_FREQUENCY)
  const float dt = 1. / (INS_PROPAGATE_FREQUENCY);
  ins_float_invariant_propagate(&gyro_f, &ins_finv_accel, dt);
#endif

  ins_finv_last_stamp = stamp;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  ACCELS_FLOAT_OF_BFP(ins_finv_accel, *accel);
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ins_float_inv.is_aligned) {
    /* convert to float */
    struct FloatRates gyro_f;
    RATES_FLOAT_OF_BFP(gyro_f, *lp_gyro);
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *lp_accel);
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *lp_mag);
    ins_float_invariant_align(&gyro_f, &accel_f, &mag_f);
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ins_float_inv_set_body_to_imu_quat(q_b2i_f);
}

#if USE_MAGNETOMETER
static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ins_float_inv.is_aligned) {
    struct FloatVect3 mag_f;
    MAGS_FLOAT_OF_BFP(mag_f, *mag);
    ins_float_invariant_update_mag(&mag_f);
  }
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  ins_float_inv.mag_h = *h;
}
#endif

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ins_float_invariant_update_gps(gps_s);
}


void ins_float_invariant_wrapper_init(void)
{
  // aligner
#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif

  ins_float_invariant_init();

 // Bind to ABI messages
  AbiBindMsgBARO_ABS(INS_FINV_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgIMU_GYRO_INT32(INS_FINV_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(INS_FINV_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_LOWPASSED(INS_FINV_IMU_ID, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(INS_FINV_IMU_ID, &body_to_imu_ev, body_to_imu_cb);
#if USE_MAGNETOMETER
  AbiBindMsgIMU_MAG_INT32(INS_FINV_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);
#endif
  AbiBindMsgGPS(INS_FINV_GPS_ID, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY && !INS_FINV_USE_UTM
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
#endif
}
