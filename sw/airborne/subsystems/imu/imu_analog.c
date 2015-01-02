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

volatile bool_t analog_imu_available;
int imu_overrun;

static struct adc_buf analog_imu_adc_buf[NB_ANALOG_IMU_ADC];

void imu_impl_init(void)
{

  analog_imu_available = FALSE;
  imu_overrun = 0;

#ifdef ADC_CHANNEL_GYRO_P
  adc_buf_channel(ADC_CHANNEL_GYRO_P, &analog_imu_adc_buf[0], ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_GYRO_Q
  adc_buf_channel(ADC_CHANNEL_GYRO_Q, &analog_imu_adc_buf[1], ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_GYRO_R
  adc_buf_channel(ADC_CHANNEL_GYRO_R, &analog_imu_adc_buf[2], ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_ACCEL_X
  adc_buf_channel(ADC_CHANNEL_ACCEL_X, &analog_imu_adc_buf[3], ADC_CHANNEL_ACCEL_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_ACCEL_Y
  adc_buf_channel(ADC_CHANNEL_ACCEL_Y, &analog_imu_adc_buf[4], ADC_CHANNEL_ACCEL_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_ACCEL_Z
  adc_buf_channel(ADC_CHANNEL_ACCEL_Z, &analog_imu_adc_buf[5], ADC_CHANNEL_ACCEL_NB_SAMPLES);
#endif

}

void imu_periodic(void)
{
  // Actual Nr of ADC measurements per channel per periodic loop
  static int last_head = 0;

  imu_overrun = analog_imu_adc_buf[0].head - last_head;
  if (imu_overrun < 0) {
    imu_overrun += ADC_CHANNEL_GYRO_NB_SAMPLES;
  }
  last_head = analog_imu_adc_buf[0].head;

  // Read All Measurements
#ifdef ADC_CHANNEL_GYRO_P
  imu.gyro_unscaled.p = analog_imu_adc_buf[0].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
#endif
#ifdef ADC_CHANNEL_GYRO_Q
  imu.gyro_unscaled.q = analog_imu_adc_buf[1].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
#endif
#ifdef ADC_CHANNEL_GYRO_R
  imu.gyro_unscaled.r = analog_imu_adc_buf[2].sum / ADC_CHANNEL_GYRO_NB_SAMPLES;
#endif
#ifdef ADC_CHANNEL_ACCEL_X
  imu.accel_unscaled.x = analog_imu_adc_buf[3].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;
#endif
#ifdef ADC_CHANNEL_ACCEL_Y
  imu.accel_unscaled.y = analog_imu_adc_buf[4].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;
#endif
#ifdef ADC_CHANNEL_ACCEL_Z
  imu.accel_unscaled.z = analog_imu_adc_buf[5].sum / ADC_CHANNEL_ACCEL_NB_SAMPLES;
#endif

  analog_imu_available = TRUE;
}

// if not all gyros are used, override the imu_scale_gyro handler
#if defined ADC_CHANNEL_GYRO_P && defined ADC_CHANNEL_GYRO_Q && ! defined ADC_CHANNEL_GYRO_R

void imu_scale_gyro(struct Imu *_imu)
{
  _imu->gyro.p = ((_imu->gyro_unscaled.p - _imu->gyro_neutral.p) * IMU_GYRO_P_SIGN * IMU_GYRO_P_SENS_NUM) /
                 IMU_GYRO_P_SENS_DEN;
  _imu->gyro.q = ((_imu->gyro_unscaled.q - _imu->gyro_neutral.q) * IMU_GYRO_Q_SIGN * IMU_GYRO_Q_SENS_NUM) /
                 IMU_GYRO_Q_SENS_DEN;
}

#elif defined ADC_CHANNEL_GYRO_P && ! defined ADC_CHANNEL_GYRO_Q && ! defined ADC_CHANNEL_GYRO_R

void imu_scale_gyro(struct Imu *_imu)
{
  _imu->gyro.p = ((_imu->gyro_unscaled.p - _imu->gyro_neutral.p) * IMU_GYRO_P_SIGN * IMU_GYRO_P_SENS_NUM) /
                 IMU_GYRO_P_SENS_DEN;
}

#endif

// if we don't have any accelerometers, set an empty imu_scale_accel handler
#if ! defined ADC_CHANNEL_ACCEL_X && ! defined ADC_CHANNEL_ACCEL_Z && ! defined ADC_CHANNEL_ACCEL_Z
void imu_scale_accel(struct Imu *_imu __attribute__((unused))) {}
#endif
