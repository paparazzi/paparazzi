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

#include "subsystems/imu.h"
#include "subsystems/abi.h"

volatile bool ADS8344_available;
uint16_t ADS8344_values[ADS8344_NB_CHANNELS];

void imu_impl_init(void)
{

  ADS8344_available = false;

  imu_crista_arch_init();

#ifdef USE_AMI601
  ami601_init();
#endif
#ifdef USE_HMC5843
  hmc5843_init();
#endif

}

void imu_periodic(void)
{

  ImuCristaArchPeriodic();
#ifdef USE_AMI601
  RunOnceEvery(10, { ami601_read(); });
#endif
#ifdef USE_HMC5843
  hmc5843_periodic();
#endif
}



#ifdef USE_AMI601
#include "peripherals/ami601.h"
#define foo_handler() {}
static inline void ImuMagEvent()
{
  AMI601Event(foo_handler);
  if (ami601_status == AMI601_DATA_AVAILABLE) {
    imu.mag_unscaled.x = ami601_values[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = ami601_values[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.z = ami601_values[IMU_MAG_Z_CHAN];
    ami601_status = AMI601_IDLE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_CRISTA_ID, now_ts, &imu.mag);
  }
}
#elif defined USE_HMC5843
#include "peripherals/hmc5843.h"
static inline void ImuMagEvent(void)
{
  hmc5843_idle_task();
  if (hmc5843.data_available) {
    imu.mag_unscaled.x = hmc5843.data.value[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = hmc5843.data.value[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.z = hmc5843.data.value[IMU_MAG_Z_CHAN];
    hmc5843.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_CRISTA_ID, now_ts, &imu.mag);
  }
}
#else
#define ImuMagEvent() {}
#endif

void imu_christa_event(void)
{
  if (ADS8344_available) {
    ADS8344_available = false;
    imu.gyro_unscaled.p = ADS8344_values[IMU_GYRO_P_CHAN];
    imu.gyro_unscaled.q = ADS8344_values[IMU_GYRO_Q_CHAN];
    imu.gyro_unscaled.r = ADS8344_values[IMU_GYRO_R_CHAN];
    imu.accel_unscaled.x = ADS8344_values[IMU_ACCEL_X_CHAN];
    imu.accel_unscaled.y = ADS8344_values[IMU_ACCEL_Y_CHAN];
    imu.accel_unscaled.z = ADS8344_values[IMU_ACCEL_Z_CHAN];
    /* spare 3, temp 7 */
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_CRISTA_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_CRISTA_ID, now_ts, &imu.accel);
  }
  ImuMagEvent();
}
