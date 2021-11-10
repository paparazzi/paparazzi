/*
 * Copyright (C) 2013-2015 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_mpu60x0_i2c.c
 * Driver for IMU with only MPU60X0 via I2C.
 */

#include <math.h>
#include "subsystems/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/i2c.h"
#include "led.h"

/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined IMU_MPU60X0_LOWPASS_FILTER && !defined  IMU_MPU60X0_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define IMU_MPU60X0_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define IMU_MPU60X0_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define IMU_MPU60X0_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define IMU_MPU60X0_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
#error Non-default PERIODIC_FREQUENCY: please define IMU_MPU60X0_LOWPASS_FILTER and IMU_MPU60X0_SMPLRT_DIV.
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU60X0_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU60X0_SMPLRT_DIV)

PRINT_CONFIG_VAR(IMU_MPU60X0_GYRO_RANGE)
PRINT_CONFIG_VAR(IMU_MPU60X0_ACCEL_RANGE)

#ifndef IMU_MPU60X0_I2C_ADDR
#define IMU_MPU60X0_I2C_ADDR MPU60X0_ADDR
#endif

struct ImuMpu60x0 imu_mpu_i2c;

void imu_mpu_i2c_init(void)
{
  mpu60x0_i2c_init(&imu_mpu_i2c.mpu, &(IMU_MPU60X0_I2C_DEV), IMU_MPU60X0_I2C_ADDR);
  // change the default configuration
  imu_mpu_i2c.mpu.config.smplrt_div = IMU_MPU60X0_SMPLRT_DIV;
  imu_mpu_i2c.mpu.config.dlpf_cfg = IMU_MPU60X0_LOWPASS_FILTER;
  imu_mpu_i2c.mpu.config.gyro_range = IMU_MPU60X0_GYRO_RANGE;
  imu_mpu_i2c.mpu.config.accel_range = IMU_MPU60X0_ACCEL_RANGE;
}

void imu_mpu_i2c_periodic(void)
{
  mpu60x0_i2c_periodic(&imu_mpu_i2c.mpu);
}

void imu_mpu_i2c_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the MPU60X0 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_mpu_i2c.mpu);
  if (imu_mpu_i2c.mpu.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_mpu_i2c.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_mpu_i2c.mpu.data_accel.vect);
    imu_mpu_i2c.mpu.data_available = false;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_MPU60X0_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_MPU60X0_ID, now_ts, &imu.accel);
  }
}
