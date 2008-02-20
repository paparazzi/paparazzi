/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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
 *
 */

#include "booz_still_detection.h"
#include "booz_imu.h"

uint8_t booz_still_detection_status;
float booz_still_detection_accel[AXIS_NB];
float booz_still_detection_gyro[AXIS_NB];
float booz_still_detection_mag[AXIS_NB];

#define BSD_BUF_LEN 128

static uint16_t bsd_accel_raw[AXIS_NB][BSD_BUF_LEN];
static uint32_t bsd_accel_raw_sum[AXIS_NB];
uint32_t bsd_accel_raw_avg[AXIS_NB];
float    bsd_accel_raw_var[AXIS_NB];

static uint16_t bsd_gyro_raw[AXIS_NB][BSD_BUF_LEN];
static uint32_t bsd_gyro_raw_sum[AXIS_NB];
static uint32_t bsd_gyro_raw_avg[AXIS_NB];
static float    bsd_gyro_raw_var[AXIS_NB];

static int16_t  bsd_mag_raw[AXIS_NB][BSD_BUF_LEN];
static int32_t  bsd_mag_raw_sum[AXIS_NB];
static int32_t  bsd_mag_raw_avg[AXIS_NB];
static float    bsd_mag_raw_var[AXIS_NB];

static uint8_t  bsd_buf_head;
static bool_t bsd_buf_filled;

static inline void bsd_set_initial_values(void);

#define BsdUpdateAverage(_sensor) {					\
    bsd_##_sensor##_raw_sum[i] -= bsd_##_sensor##_raw[i][bsd_buf_head];	\
    bsd_##_sensor##_raw[i][bsd_buf_head] = imu_##_sensor##_raw[i];	\
    bsd_##_sensor##_raw_sum[i] += imu_##_sensor##_raw[i];		\
  }

void booz_still_detection_init(void) {
  bsd_buf_filled = FALSE;
  bsd_buf_head = 0;
  booz_still_detection_status = BSD_STATUS_UNINIT;
}

void booz_still_detection_run(void) {
  if (booz_still_detection_status == BSD_STATUS_LOCKED)
    return;
  /* update sliding average of sensors */
  bsd_buf_head++;
  if (bsd_buf_head >= BSD_BUF_LEN) {
    bsd_buf_filled = TRUE;
    bsd_buf_head = 0;
  } 
  
  uint8_t i, j;
  for (i=0; i<AXIS_NB; i++) {
    BsdUpdateAverage(accel);
    BsdUpdateAverage(gyro);
    BsdUpdateAverage(mag);
  }
 
  /* update sensors variance */
  if (bsd_buf_filled) {
    for (i=0; i<AXIS_NB; i++) {  
      bsd_accel_raw_avg[i] = bsd_accel_raw_sum[i] / BSD_BUF_LEN;
      bsd_gyro_raw_avg[i] = bsd_gyro_raw_sum[i] / BSD_BUF_LEN;
      bsd_mag_raw_avg[i] = bsd_mag_raw_sum[i] / BSD_BUF_LEN;

      bsd_accel_raw_var[i] = 0.;
      bsd_gyro_raw_var[i] = 0.;
      bsd_mag_raw_var[i] = 0.;

      for (j=0; j<BSD_BUF_LEN; j++) {
	int32_t diff;
	diff = bsd_gyro_raw[i][j] - bsd_gyro_raw_avg[i];
	bsd_gyro_raw_var[i] += (float)(diff*diff);
	diff = bsd_accel_raw[i][j] - bsd_accel_raw_avg[i];
	bsd_accel_raw_var[i] += (float)(diff*diff);
	diff = bsd_mag_raw[i][j] - bsd_mag_raw_avg[i];
	bsd_mag_raw_var[i] += (float)(diff*diff);
      }
      bsd_gyro_raw_var[i] /= (float)BSD_BUF_LEN;
      bsd_accel_raw_var[i] /= (float)BSD_BUF_LEN;
      bsd_mag_raw_var[i] /= (float)BSD_BUF_LEN;
    }
    /* check vehicle still */
    if (bsd_accel_raw_var[AXIS_X] < BSD_ACCEL_RAW_MAX_VAR &&
	bsd_accel_raw_var[AXIS_Y] < BSD_ACCEL_RAW_MAX_VAR &&
	bsd_accel_raw_var[AXIS_Z] < BSD_ACCEL_RAW_MAX_VAR )
      bsd_set_initial_values();
  }
}

static inline void bsd_set_initial_values(void) {
      booz_still_detection_status = BSD_STATUS_LOCKED;
      BoozImuScaleSensor(booz_still_detection_accel, IMU_ACCEL, bsd_accel_raw_avg);
      BoozImuScaleSensor(booz_still_detection_gyro,  IMU_GYRO,  bsd_gyro_raw_avg);
      BoozImuScaleSensor(booz_still_detection_mag,   IMU_MAG,   bsd_mag_raw_avg);
}
