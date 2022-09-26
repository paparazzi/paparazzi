/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/imu/filter_1euro_imu.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Prefiltering for IMU data using 1euro filter
 */

#include "modules/imu/filter_1euro_imu.h"
#include "filters/1e_filter.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"

/** Enable by default */
#ifndef FILTER_1EURO_ENABLED
#define FILTER_1EURO_ENABLED TRUE
#endif

/** Default gyro min cutoff freq */
#ifndef FILTER_1EURO_GYRO_MINCUTOFF
#define FILTER_1EURO_GYRO_MINCUTOFF 10.f
#endif

/** Default gyro beta coef */
#ifndef FILTER_1EURO_GYRO_BETA
#define FILTER_1EURO_GYRO_BETA 0.1f
#endif

/** Default gyro dcutoff (not recommanded to change) */
#ifndef FILTER_1EURO_GYRO_DCUTOFF
#define FILTER_1EURO_GYRO_DCUTOFF 1.f
#endif

/** Default accel min cutoff freq */
#ifndef FILTER_1EURO_ACCEL_MINCUTOFF
#define FILTER_1EURO_ACCEL_MINCUTOFF 0.1f
#endif

/** Default accel beta coef */
#ifndef FILTER_1EURO_ACCEL_BETA
#define FILTER_1EURO_ACCEL_BETA 0.01f
#endif

/** Default accel dcutoff (not recommanded to change) */
#ifndef FILTER_1EURO_ACCEL_DCUTOFF
#define FILTER_1EURO_ACCEL_DCUTOFF 1.f
#endif

/** Auto freq if not defined */
#ifndef FILTER_1EURO_FREQ
#if defined AHRS_PROPAGATE_FREQUENCY
#define FILTER_1EURO_FREQ AHRS_PROPAGATE_FREQUENCY
PRINT_CONFIG_VAR(FILTER_1EURO_FREQ)
#elif defined INS_PROPAGATE_FREQUENCY
#define FILTER_1EURO_FREQ INS_PROPAGATE_FREQUENCY
PRINT_CONFIG_VAR(FILTER_1EURO_FREQ)
#else
PRINT_CONFIG_MSG("FILTER_1EURO_FREQ not defined, using timestamp")
// init freq at periodic frequency, value can't be zero
#define FILTER_1EURO_FREQ PERIODIC_FREQUENCY
#endif
#endif

/**
 * configuration structure
 */
struct Filter1eImu filter_1e_imu;

/**
 * array of 1 euro filters for gyrometer
 */
static struct OneEuroFilter gyro_1e[3];

/**
 * array of 1 euro filters for accelerometer
 */
static struct OneEuroFilter accel_1e[3];

/**
 * ABI bindings
 *
 * by default bind to all IMU raw data and send filtered data
 * receivers (AHRS, INS) should bind to this prefilter module
 */
/** IMU (gyro, accel) */
#ifndef IMU_F1E_BIND_ID
#define IMU_F1E_BIND_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(IMU_F1E_BIND_ID)

static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev; // only passthrough

static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro, uint8_t samples)
{
  if (sender_id == IMU_F1E_ID) {
    return; // don't process own data
  }

  if (filter_1e_imu.enabled && samples > 0) {
    struct FloatRates gyro_f;
    RATES_FLOAT_OF_BFP(gyro_f, gyro[samples-1]); // For now only filter last value
    // compute filters
#ifdef FILTER_1EURO_FREQ
    gyro_f.p = update_1e_filter(&gyro_1e[0], gyro_f.p);
    gyro_f.q = update_1e_filter(&gyro_1e[1], gyro_f.q);
    gyro_f.r = update_1e_filter(&gyro_1e[2], gyro_f.r);
#else
    // use timestamp
    gyro_f.p = update_1e_filter_at_time(&gyro_1e[0], gyro_f.p, stamp);
    gyro_f.q = update_1e_filter_at_time(&gyro_1e[1], gyro_f.q, stamp);
    gyro_f.r = update_1e_filter_at_time(&gyro_1e[2], gyro_f.r, stamp);
#endif
    // send filtered data
    struct Int32Rates gyro_i;
    RATES_BFP_OF_REAL(gyro_i, gyro_f);
    AbiSendMsgIMU_GYRO_RAW(IMU_F1E_ID, stamp, &gyro_i, 1);
  } else {
    AbiSendMsgIMU_GYRO_RAW(IMU_F1E_ID, stamp, gyro, samples);
  }
}

static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel, uint8_t samples)
{
  if (sender_id == IMU_F1E_ID) {
    return; // don't process own data
  }

  if (filter_1e_imu.enabled && samples > 0) {
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *accel);
    // compute filters
#ifdef FILTER_1EURO_FREQ
    accel_f.x = update_1e_filter(&accel_1e[0], accel_f.x);
    accel_f.y = update_1e_filter(&accel_1e[1], accel_f.y);
    accel_f.z = update_1e_filter(&accel_1e[2], accel_f.z);
#else
    // use timestamp
    accel_f.x = update_1e_filter_at_time(&accel_1e[0], accel_f.x, stamp);
    accel_f.y = update_1e_filter_at_time(&accel_1e[1], accel_f.y, stamp);
    accel_f.z = update_1e_filter_at_time(&accel_1e[2], accel_f.z, stamp);
#endif
    // send filtered data
    struct Int32Vect3 accel_i;
    ACCELS_BFP_OF_REAL(accel_i, accel_f);
    AbiSendMsgIMU_ACCEL_RAW(IMU_F1E_ID, stamp, &accel_i, 1);
  } else {
    AbiSendMsgIMU_ACCEL_RAW(IMU_F1E_ID, stamp, accel, samples);
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp, struct Int32Vect3 *mag)
{
  if (sender_id == IMU_F1E_ID) {
    return; // don't process own data
  }

  AbiSendMsgIMU_MAG_RAW(IMU_F1E_ID, stamp, mag);
}

/**
 * Init and bindings
 */
void filter_1euro_imu_init(void)
{
  filter_1e_imu.enabled         = FILTER_1EURO_ENABLED;
  filter_1e_imu.gyro_mincutoff  = FILTER_1EURO_GYRO_MINCUTOFF;
  filter_1e_imu.gyro_beta       = FILTER_1EURO_GYRO_BETA;
  filter_1e_imu.gyro_dcutoff    = FILTER_1EURO_GYRO_DCUTOFF;
  filter_1e_imu.accel_mincutoff = FILTER_1EURO_ACCEL_MINCUTOFF;
  filter_1e_imu.accel_beta      = FILTER_1EURO_ACCEL_BETA;
  filter_1e_imu.accel_dcutoff   = FILTER_1EURO_ACCEL_DCUTOFF;

  for (int i = 0; i < 3; i++) {
    init_1e_filter(&gyro_1e[i],
        FILTER_1EURO_FREQ,
        filter_1e_imu.gyro_mincutoff,
        filter_1e_imu.gyro_beta,
        filter_1e_imu.gyro_dcutoff);
    init_1e_filter(&accel_1e[i],
        FILTER_1EURO_FREQ,
        filter_1e_imu.accel_mincutoff,
        filter_1e_imu.accel_beta,
        filter_1e_imu.accel_dcutoff);
  }

  AbiBindMsgIMU_GYRO_RAW(IMU_F1E_BIND_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_RAW(IMU_F1E_BIND_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_RAW(IMU_F1E_BIND_ID, &mag_ev, mag_cb);
}

/**
 * settings handlers
 */

void filter_1euro_imu_reset(float enabled)
{
  filter_1e_imu.enabled = enabled;
  for (int i = 0; i < 3; i++) {
    reset_1e_filter(&gyro_1e[i]);
    reset_1e_filter(&accel_1e[i]);
  }
}

void filter_1euro_imu_update_gyro_mincutoff(float mincutoff)
{
  filter_1e_imu.gyro_mincutoff = mincutoff;
  for (int i = 0; i < 3; i++) {
    gyro_1e[i].mincutoff = mincutoff;
  }
}

void filter_1euro_imu_update_gyro_beta(float beta)
{
  filter_1e_imu.gyro_beta = beta;
  for (int i = 0; i < 3; i++) {
    gyro_1e[i].beta = beta;
  }
}

void filter_1euro_imu_update_gyro_dcutoff(float dcutoff)
{
  filter_1e_imu.gyro_dcutoff = dcutoff;
  for (int i = 0; i < 3; i++) {
    gyro_1e[i].dcutoff = dcutoff;
  }
}

void filter_1euro_imu_update_accel_mincutoff(float mincutoff)
{
  filter_1e_imu.accel_mincutoff = mincutoff;
  for (int i = 0; i < 3; i++) {
    accel_1e[i].mincutoff = mincutoff;
  }
}

void filter_1euro_imu_update_accel_beta(float beta)
{
  filter_1e_imu.accel_beta = beta;
  for (int i = 0; i < 3; i++) {
    accel_1e[i].beta = beta;
  }
}

void filter_1euro_imu_update_accel_dcutoff(float dcutoff)
{
  filter_1e_imu.accel_dcutoff = dcutoff;
  for (int i = 0; i < 3; i++) {
    accel_1e[i].dcutoff = dcutoff;
  }
}
