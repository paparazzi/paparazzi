/*
 * $Id$
 *
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

#include "ahrs_aligner.h"

#include <stdlib.h> /* for abs() */
#include "subsystems/imu.h"
#include "led.h"

struct AhrsAligner ahrs_aligner;

#define SAMPLES_NB PERIODIC_FREQUENCY

static struct Int32Rates gyro_sum;
static struct Int32Vect3 accel_sum;
static struct Int32Vect3 mag_sum;
static int32_t ref_sensor_samples[SAMPLES_NB];
static uint32_t samples_idx;

void ahrs_aligner_init(void) {

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
#ifndef LOW_NOISE_TIME
#define LOW_NOISE_TIME          5
#endif

void ahrs_aligner_run(void) {

  RATES_ADD(gyro_sum,  imu.gyro);
  VECT3_ADD(accel_sum, imu.accel);
  VECT3_ADD(mag_sum,   imu.mag);

  ref_sensor_samples[samples_idx] = imu.accel.z;
  samples_idx++;

#ifdef AHRS_ALIGNER_LED
  RunOnceEvery(50, {LED_TOGGLE(AHRS_ALIGNER_LED);});
#endif

  if (samples_idx >= SAMPLES_NB) {
    int32_t avg_ref_sensor = accel_sum.z;
    if ( avg_ref_sensor >= 0)
      avg_ref_sensor += SAMPLES_NB / 2;
    else
      avg_ref_sensor -= SAMPLES_NB / 2;
    avg_ref_sensor /= SAMPLES_NB;

    ahrs_aligner.noise = 0;
    int i;
    for (i=0; i<SAMPLES_NB; i++) {
      int32_t diff = ref_sensor_samples[i] - avg_ref_sensor;
      ahrs_aligner.noise += abs(diff);
    }

    RATES_SDIV(ahrs_aligner.lp_gyro,  gyro_sum,  SAMPLES_NB);
    VECT3_SDIV(ahrs_aligner.lp_accel, accel_sum, SAMPLES_NB);
    VECT3_SDIV(ahrs_aligner.lp_mag,   mag_sum,   SAMPLES_NB);

    INT_RATES_ZERO(gyro_sum);
    INT_VECT3_ZERO(accel_sum);
    INT_VECT3_ZERO(mag_sum);
    samples_idx = 0;

    if (ahrs_aligner.noise < LOW_NOISE_THRESHOLD)
      ahrs_aligner.low_noise_cnt++;
    else
      if ( ahrs_aligner.low_noise_cnt > 0)
        ahrs_aligner.low_noise_cnt--;

    if (ahrs_aligner.low_noise_cnt > LOW_NOISE_TIME) {
      ahrs_aligner.status = AHRS_ALIGNER_LOCKED;
#ifdef AHRS_ALIGNER_LED
      LED_ON(AHRS_ALIGNER_LED);
#endif
    }
  }

}
