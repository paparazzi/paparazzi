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

#include "booz_imu.h"

#include "booz_debug.h"

/* calibrated sensors */
float   imu_accel[AXIS_NB]; /* accelerometers in m/s2           */
float   imu_gyro[AXIS_NB];  /* gyros in rad/s                   */
float   imu_mag[AXIS_NB];   /* magnetometer in arbitrary unit   */
float   imu_pressure;       /* static pressure in pascals       */

/* raw sensors */
uint16_t imu_accel_raw[AXIS_NB];
uint16_t imu_gyro_raw[AXIS_NB];
int16_t  imu_mag_raw[AXIS_NB];
uint32_t imu_pressure_raw;

/* internal ADCs */
struct adc_buf buf_ax;
struct adc_buf buf_ay;
struct adc_buf buf_az;

uint8_t booz_imu_status;

#define IMU_ERR_OVERUN 0

void booz_imu_init(void) {
  uint8_t i;
  for (i=0; i<AXIS_NB; i++) {
    imu_accel_raw[i] = 0;
    imu_gyro_raw[i] = 0;
    imu_mag_raw[i] = 0;
    imu_accel[i] = 0.;
    imu_gyro[i] = 0.;
    imu_mag[i] = 0.;
  }
  imu_pressure_raw = 0;
  imu_pressure = 0.;
  booz_imu_status = BOOZ_IMU_STA_IDLE;

  adc_buf_channel(IMU_ACCEL_X_CHAN, &buf_ax, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(IMU_ACCEL_Y_CHAN, &buf_ay, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(IMU_ACCEL_Z_CHAN, &buf_az, DEFAULT_AV_NB_SAMPLE);
  adc_init();
  max1167_init(); 
  micromag_init();
  scp1000_init();
  booz_imu_hw_init();
}

void booz_imu_periodic(void) {
  ASSERT((booz_imu_status == BOOZ_IMU_STA_IDLE), DEBUG_IMU, IMU_ERR_OVERUN);
  booz_imu_status = BOOZ_IMU_STA_MEASURING_GYRO;
  max1167_read();
}
