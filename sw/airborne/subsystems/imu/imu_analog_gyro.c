/*
 * Copyright (C) 2011 The Paparazzi Team
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

#include BOARD_CONFIG
#include "imu_analog_gyro.h"
#include "mcu_periph/adc.h"


volatile bool_t imu_analog_gyro_available;
int imu_overrun;

static struct adc_buf imu_gyro_adc_buf[NB_IMU_GYRO_ADC];

//#if defined ADC_CHANNEL_GYRO_TEMP
//static struct adc_buf buf_temp;
//#endif

void imu_impl_init(void) {

  imu_analog_gyro_available = FALSE;
  imu_overrun = 0;

#ifdef ADC_CHANNEL_GYRO_P
  adc_buf_channel(ADC_CHANNEL_GYRO_P, &imu_gyro_adc_buf[0], ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_GYRO_Q
  adc_buf_channel(ADC_CHANNEL_GYRO_Q, &imu_gyro_adc_buf[1], ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif

//#ifdef ADC_CHANNEL_GYRO_P_TEMP
//  adc_buf_channel(ADC_CHANNEL_GYRO_P_TEMP, &buf_temp, ADC_CHANNEL_GYRO_NB_SAMPLES);
//#endif

}

void imu_periodic(void) {
  // Actual Nr of ADC measurements per channel per periodic loop
  static int last_head = 0;

  imu_overrun = imu_gyro_adc_buf[0].head - last_head;
  if (imu_overrun < 0)
    imu_overrun += ADC_CHANNEL_GYRO_NB_SAMPLES;
  last_head = imu_gyro_adc_buf[0].head;

  // Read All Measurements
#ifdef ADC_CHANNEL_GYRO_P
  imu.gyro_unscaled.p = imu_gyro_adc_buf[0].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
#endif
#ifdef ADC_CHANNEL_GYRO_Q
  imu.gyro_unscaled.q = imu_gyro_adc_buf[1].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
#endif

  imu_analog_gyro_available = TRUE;
}
