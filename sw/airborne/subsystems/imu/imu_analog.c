/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include "imu_analog.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"

volatile bool_t analog_imu_available;
int imu_overrun;

static struct adc_buf analog_imu_adc_buf[NB_ANALOG_IMU_ADC];

void imu_impl_init(void) {

  analog_imu_available = FALSE;
  imu_overrun = 0;

  adc_buf_channel(ADC_CHANNEL_GYRO_P, &analog_imu_adc_buf[0], ADC_CHANNEL_GYRO_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_GYRO_Q, &analog_imu_adc_buf[1], ADC_CHANNEL_GYRO_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_GYRO_R, &analog_imu_adc_buf[2], ADC_CHANNEL_GYRO_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_ACCEL_X, &analog_imu_adc_buf[3], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_ACCEL_Y, &analog_imu_adc_buf[4], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_ACCEL_Z, &analog_imu_adc_buf[5], ADC_CHANNEL_ACCEL_NB_SAMPLES);

}

void imu_periodic(void) {
  // Actual Nr of ADC measurements per channel per periodic loop
  static int last_head = 0;

  imu_overrun = analog_imu_adc_buf[0].head - last_head;
  if (imu_overrun < 0)
    imu_overrun += ADC_CHANNEL_GYRO_NB_SAMPLES;
  last_head = analog_imu_adc_buf[0].head;

  // Read All Measurements
  imu.gyro_unscaled.p = analog_imu_adc_buf[0].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
  imu.gyro_unscaled.q = analog_imu_adc_buf[1].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
  imu.gyro_unscaled.r = analog_imu_adc_buf[2].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
  imu.accel_unscaled.x = analog_imu_adc_buf[3].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;
  imu.accel_unscaled.y = analog_imu_adc_buf[4].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;
  imu.accel_unscaled.z = analog_imu_adc_buf[5].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;

  analog_imu_available = TRUE;
}
