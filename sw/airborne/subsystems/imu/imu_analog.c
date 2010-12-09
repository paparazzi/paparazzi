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

#include "imu_analog.h"
#include "generated/airframe.h"
#include "mcu_periph/adc.h"
#include "subsystems/imu.h"
#include "mcu_periph/uart.h"

volatile bool_t analog_imu_available;
uint16_t analog_imu_values[NB_ANALOG_IMU_ADC];

static struct adc_buf analog_imu_adc_buf[NB_ANALOG_IMU_ADC];

void imu_impl_init(void) {

  analog_imu_available = FALSE;

  adc_buf_channel(ADC_CHANNEL_GYRO_P, &analog_imu_adc_buf[0], ADC_CHANNEL_GYRO_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_GYRO_Q, &analog_imu_adc_buf[1], ADC_CHANNEL_GYRO_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_GYRO_R, &analog_imu_adc_buf[2], ADC_CHANNEL_GYRO_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_ACCEL_X, &analog_imu_adc_buf[3], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_ACCEL_Y, &analog_imu_adc_buf[4], ADC_CHANNEL_ACCEL_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_ACCEL_Z, &analog_imu_adc_buf[5], ADC_CHANNEL_ACCEL_NB_SAMPLES);

}

void imu_periodic(void) {
  uint8_t i;
  for(i = 0; i < NB_ANALOG_IMU_ADC; i++) {
    analog_imu_values[i] = analog_imu_adc_buf[i].sum / analog_imu_adc_buf[i].av_nb_sample;
  }

  analog_imu_available = TRUE;
}
