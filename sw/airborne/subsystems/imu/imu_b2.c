/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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
 * @file subsystems/imu/imu_b2.c
 *
 * Driver for the Booz2 IMUs.
 *
 * Analog gyros and accelerometers are read via MAX1168 16-bit SPI ADC.
 * Depending on version, different I2C or SPI magnetometers are used.
 */

#include "subsystems/imu.h"

struct ImuBooz2 imu_b2;

PRINT_CONFIG_VAR(IMU_B2_MAG_TYPE)

void imu_impl_init(void)
{

  max1168_init();
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
  ms2100_init(&ms2100, &(MS2100_SPI_DEV), MS2100_SLAVE_IDX);
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
  ami601_init();
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC5843
  hmc5843_init();
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC58XX
  hmc58xx_init(&imu_b2.mag_hmc, &(IMU_B2_I2C_DEV), HMC58XX_ADDR);
  // Booz2 v1.2 has HMC5843
  imu_b2.mag_hmc.type = HMC_TYPE_5843;
#endif

}

#include "led.h"
void imu_periodic(void)
{

  // read adc
  Max1168Periodic();
  // read mag
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
  ms2100_periodic(&ms2100);
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
  RunOnceEvery(10, { ami601_read(); });
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC58XX
  RunOnceEvery(5, hmc58xx_periodic(&imu_b2.mag_hmc));
#endif

}

#if defined IMU_MAG_45_HACK
void imu_scale_mag(struct Imu *_imu)
{
  int32_t msx = ((_imu->mag_unscaled.x - _imu->mag_neutral.x) * IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN;
  int32_t msy = ((_imu->mag_unscaled.y - _imu->mag_neutral.y) * IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN;
  _imu->mag.x = msx - msy;
  _imu->mag.y = msx + msy;
  _imu->mag.z = ((_imu->mag_unscaled.z - _imu->mag_neutral.z) * IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN;
}
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_NONE
void imu_scale_mag(struct Imu *_imu __attribute__((unused))) {}
#endif



/** Event functions for imu_b2.
 */
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"

#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
static inline void ImuMagEvent(void)
{
  ms2100_event(&ms2100);
  if (ms2100.status == MS2100_DATA_AVAILABLE) {
    imu.mag_unscaled.x = ms2100.data.value[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = ms2100.data.value[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.z = ms2100.data.value[IMU_MAG_Z_CHAN];
    ms2100.status = MS2100_IDLE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_B2_ID, get_sys_time_usec(), &imu.mag);
  }
}
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
static inline void foo_handler(void) {}
static inline void ImuMagEvent(void)
{
  AMI601Event(foo_handler);
  if (ami601_status == AMI601_DATA_AVAILABLE) {
    imu.mag_unscaled.x = ami601_values[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = ami601_values[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.z = ami601_values[IMU_MAG_Z_CHAN];
    ami601_status = AMI601_IDLE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_B2_ID, get_sys_time_usec(), &imu.mag);
  }
}
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC5843
static inline void foo_handler(void) {}
static inline void ImuMagEvent(void)
{
  hmc5843_idle_task();
  if (hmc5843.data_available) {
    imu.mag_unscaled.x = hmc5843.data.value[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = hmc5843.data.value[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.z = hmc5843.data.value[IMU_MAG_Z_CHAN];
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_B2_ID, get_sys_time_usec(), &imu.mag);
    hmc5843.data_available = false;
  }
}
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC58XX
static inline void ImuMagEvent(void)
{
  hmc58xx_event(&imu_b2.mag_hmc);
  if (imu_b2.mag_hmc.data_available) {
    imu.mag_unscaled.x = imu_b2.mag_hmc.data.value[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = imu_b2.mag_hmc.data.value[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.z = imu_b2.mag_hmc.data.value[IMU_MAG_Z_CHAN];
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_B2_ID, get_sys_time_usec(), &imu.mag);
    imu_b2.mag_hmc.data_available = false;
  }
}
#else
#define ImuMagEvent() {}
#endif


void imu_b2_event(void)
{
  max1168_event();
  if (max1168_status == MAX1168_DATA_AVAILABLE) {
    uint32_t now_ts = get_sys_time_usec();
    imu.gyro_unscaled.p  = max1168_values[IMU_GYRO_P_CHAN];
    imu.gyro_unscaled.q  = max1168_values[IMU_GYRO_Q_CHAN];
    imu.gyro_unscaled.r  = max1168_values[IMU_GYRO_R_CHAN];
    imu.accel_unscaled.x = max1168_values[IMU_ACCEL_X_CHAN];
    imu.accel_unscaled.y = max1168_values[IMU_ACCEL_Y_CHAN];
    imu.accel_unscaled.z = max1168_values[IMU_ACCEL_Z_CHAN];
    max1168_status = MAX1168_IDLE;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_ASPIRIN2_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_ASPIRIN2_ID, now_ts, &imu.accel);
  }
  ImuMagEvent();
}
