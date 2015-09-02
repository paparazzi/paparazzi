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

#ifndef AHRS_DCM_OUTPUT_ENABLED
#define AHRS_DCM_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_DCM_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool_t ahrs_dcm_output_enabled;
static uint32_t ahrs_dcm_last_stamp;

static void set_body_orientation_and_rates(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"

#ifndef AHRS_DCM_FILTER_ID
#define AHRS_DCM_FILTER_ID 6
#endif

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t id = AHRS_DCM_FILTER_ID;
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_dcm.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_dcm_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &id, &mde, &val);
}
#endif

/** ABI binding for IMU data.
 * Used for gyro and accel ABI messages.
 */
#ifndef AHRS_DCM_IMU_ID
#define AHRS_DCM_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(AHRS_DCM_IMU_ID)
/** ABI binding for magnetometer data.
 * Used for IMU_MAG_INT32 ABI messages.
 */
#ifndef AHRS_DCM_MAG_ID
#define AHRS_DCM_MAG_ID AHRS_DCM_IMU_ID
#endif
PRINT_CONFIG_VAR(AHRS_DCM_MAG_ID)
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;
static abi_event gps_ev;


static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  ahrs_dcm_last_stamp = stamp;
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS dcm propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_dcm.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_dcm_propagate(gyro, dt);
    set_body_orientation_and_rates();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS dcm propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_dcm.is_aligned) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_dcm_propagate(gyro, dt);
    set_body_orientation_and_rates();
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
    if (ahrs_dcm_align(lp_gyro, lp_accel, lp_mag)) {
      set_body_orientation_and_rates();
    }
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_dcm_set_body_to_imu_quat(q_b2i_f);
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ahrs_dcm_update_gps(gps_s);
}

static bool_t ahrs_dcm_enable_output(bool_t enable)
{
  ahrs_dcm_output_enabled = enable;
  return ahrs_dcm_output_enabled;
}

static bool_t ahrs_dcm_is_aligned(void)
{
  return ahrs_dcm.is_aligned;
}

/**
 * Compute body orientation and rates from imu orientation and rates
 */
static void set_body_orientation_and_rates(void)
{
  if (ahrs_dcm_output_enabled) {
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ahrs_dcm.body_to_imu);

    struct FloatRates body_rate;
    float_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_dcm.imu_rate);
    stateSetBodyRates_f(&body_rate);

    struct FloatRMat ltp_to_imu_rmat, ltp_to_body_rmat;
    float_rmat_of_eulers(&ltp_to_imu_rmat, &ahrs_dcm.ltp_to_imu_euler);
    float_rmat_comp_inv(&ltp_to_body_rmat, &ltp_to_imu_rmat, body_to_imu_rmat);

    stateSetNedToBodyRMat_f(&ltp_to_body_rmat);
  }
}

void ahrs_dcm_register(void)
{
  ahrs_dcm_output_enabled = AHRS_DCM_OUTPUT_ENABLED;
  ahrs_dcm_init();
  ahrs_register_impl(ahrs_dcm_enable_output,ahrs_dcm_is_aligned);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(AHRS_DCM_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_DCM_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(AHRS_DCM_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "STATE_FILTER_STATUS", send_filter_status);
#endif
}
