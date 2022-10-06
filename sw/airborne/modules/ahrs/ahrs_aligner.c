/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file modules/ahrs/ahrs_aligner.c
 *
 * Low-pass IMU measurements at startup to align the AHRS.
 *
 */

#include "ahrs_aligner.h"

#include <stdlib.h> /* for abs() */
#include "modules/imu/imu.h"
#include "led.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"

struct AhrsAligner ahrs_aligner;

#ifndef AHRS_ALIGNER_SAMPLES_NB
#define AHRS_ALIGNER_SAMPLES_NB 100
#endif

static struct Int32Rates gyro_sum;
static struct Int32Vect3 accel_sum;
static struct Int32Vect3 mag_sum;
static int32_t ref_sensor_samples[AHRS_ALIGNER_SAMPLES_NB];
static uint32_t samples_idx;

#ifndef AHRS_ALIGNER_IMU_ID
#define AHRS_ALIGNER_IMU_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro __attribute__((unused)))
{
  if (ahrs_aligner.status != AHRS_ALIGNER_LOCKED) {
    ahrs_aligner_run();
  }
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_aligner(struct transport_tx *trans, struct link_device *dev)
{
  struct imu_gyro_t *gyro = imu_get_gyro(AHRS_ALIGNER_IMU_ID, false);
  if(gyro != NULL) {
    pprz_msg_send_FILTER_ALIGNER(trans, dev, AC_ID,
                                &ahrs_aligner.lp_gyro.p,
                                &ahrs_aligner.lp_gyro.q,
                                &ahrs_aligner.lp_gyro.r,
                                &gyro->scaled.p,
                                &gyro->scaled.q,
                                &gyro->scaled.r,
                                &ahrs_aligner.noise,
                                &ahrs_aligner.low_noise_cnt,
                                &ahrs_aligner.status);
  }
}
#endif

void ahrs_aligner_init(void)
{
  ahrs_aligner_restart();

  // for now: only bind to gyro message and still read from global imu struct
  AbiBindMsgIMU_GYRO(AHRS_ALIGNER_IMU_ID, &gyro_ev, gyro_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FILTER_ALIGNER, send_aligner);
#endif
}

void ahrs_aligner_restart(void)
{
  ahrs_aligner.status = AHRS_ALIGNER_RUNNING;
  INT_RATES_ZERO(gyro_sum);
  INT_VECT3_ZERO(accel_sum);
  INT_VECT3_ZERO(mag_sum);
  samples_idx = 0;
  ahrs_aligner.noise = 0;
  ahrs_aligner.low_noise_cnt = 0;
}

#ifndef LOW_NOISE_THRESHOLD
#define LOW_NOISE_THRESHOLD 90000
#endif
/** Number of cycles (100 samples each) with low noise */
#ifndef LOW_NOISE_TIME
#define LOW_NOISE_TIME 5
#endif

void ahrs_aligner_run(void)
{
  struct imu_gyro_t *gyro = imu_get_gyro(AHRS_ALIGNER_IMU_ID, false);
  struct imu_accel_t *accel = imu_get_accel(AHRS_ALIGNER_IMU_ID, false);
  struct imu_mag_t *mag = imu_get_mag(AHRS_ALIGNER_IMU_ID, false);

  // Could not find all sensors
  if(gyro == NULL || accel == NULL)
    return;

  RATES_ADD(gyro_sum, gyro->scaled);
  VECT3_ADD(accel_sum, accel->scaled);
  if(mag != NULL)
    VECT3_ADD(mag_sum, mag->scaled);

  ref_sensor_samples[samples_idx] = accel->scaled.z;
  samples_idx++;

#ifdef AHRS_ALIGNER_LED
  RunOnceEvery(50, {LED_TOGGLE(AHRS_ALIGNER_LED);});
#endif

  if (samples_idx >= AHRS_ALIGNER_SAMPLES_NB) {
    int32_t avg_ref_sensor = accel_sum.z;
    if (avg_ref_sensor >= 0) {
      avg_ref_sensor += AHRS_ALIGNER_SAMPLES_NB / 2;
    } else {
      avg_ref_sensor -= AHRS_ALIGNER_SAMPLES_NB / 2;
    }
    avg_ref_sensor /= AHRS_ALIGNER_SAMPLES_NB;

    ahrs_aligner.noise = 0;
    int i;
    for (i = 0; i < AHRS_ALIGNER_SAMPLES_NB; i++) {
      int32_t diff = ref_sensor_samples[i] - avg_ref_sensor;
      ahrs_aligner.noise += abs(diff);
    }

    RATES_SDIV(ahrs_aligner.lp_gyro,  gyro_sum,  AHRS_ALIGNER_SAMPLES_NB);
    VECT3_SDIV(ahrs_aligner.lp_accel, accel_sum, AHRS_ALIGNER_SAMPLES_NB);
    VECT3_SDIV(ahrs_aligner.lp_mag,   mag_sum,   AHRS_ALIGNER_SAMPLES_NB);

    INT_RATES_ZERO(gyro_sum);
    INT_VECT3_ZERO(accel_sum);
    INT_VECT3_ZERO(mag_sum);
    samples_idx = 0;

    if (ahrs_aligner.noise < LOW_NOISE_THRESHOLD) {
      ahrs_aligner.low_noise_cnt++;
    } else if (ahrs_aligner.low_noise_cnt > 0) {
      ahrs_aligner.low_noise_cnt--;
    }

    if (ahrs_aligner.low_noise_cnt > LOW_NOISE_TIME) {
      ahrs_aligner.status = AHRS_ALIGNER_LOCKED;
#ifdef AHRS_ALIGNER_LED
      LED_ON(AHRS_ALIGNER_LED);
#endif
      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgIMU_LOWPASSED(AHRS_ALIGNER_ID, now_ts, &ahrs_aligner.lp_gyro,
                              &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);
    }
  }

}
