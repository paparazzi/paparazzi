/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/imu/imu_mpu9250_i2c.h
 *
 * IMU driver for the MPU9250 using I2C
 *
 */

#ifndef IMU_MPU9250_I2C_H
#define IMU_MPU9250_I2C_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/mpu9250_i2c.h"


/** default gyro sensitivy and neutral from the datasheet
 * MPU with 1000 deg/s has 32.8 LSB/(deg/s)
 * sens = 1/32.8 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/32.8 * pi/180 * 4096 = 2.17953
 I*/
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
// FIXME
#define IMU_GYRO_P_SENS 2.17953
#define IMU_GYRO_P_SENS_NUM 18271
#define IMU_GYRO_P_SENS_DEN 8383
#define IMU_GYRO_Q_SENS 2.17953
#define IMU_GYRO_Q_SENS_NUM 18271
#define IMU_GYRO_Q_SENS_DEN 8383
#define IMU_GYRO_R_SENS 2.17953
#define IMU_GYRO_R_SENS_NUM 18271
#define IMU_GYRO_R_SENS_DEN 8383
#endif

/** default accel sensitivy from the datasheet
 * MPU with 8g has 4096 LSB/g
 * sens = 9.81 [m/s^2] / 4096 [LSB/g] * 2^INT32_ACCEL_FRAC = 2.4525
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
// FIXME
#define IMU_ACCEL_X_SENS 2.4525
#define IMU_ACCEL_X_SENS_NUM 981
#define IMU_ACCEL_X_SENS_DEN 400
#define IMU_ACCEL_Y_SENS 2.4525
#define IMU_ACCEL_Y_SENS_NUM 981
#define IMU_ACCEL_Y_SENS_DEN 400
#define IMU_ACCEL_Z_SENS 2.4525
#define IMU_ACCEL_Z_SENS_NUM 981
#define IMU_ACCEL_Z_SENS_DEN 400
#endif


struct ImuMpu9250 {
  volatile bool_t gyro_valid;
  volatile bool_t accel_valid;
  volatile bool_t mag_valid;
  struct Mpu9250_I2c mpu;
};

extern struct ImuMpu9250 imu_mpu9250;

extern void imu_mpu9250_event(void);

static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
  imu_mpu9250_event();
  if (imu_mpu9250.accel_valid) {
    imu_mpu9250.accel_valid = FALSE;
    _accel_handler();
  }
  if (imu_mpu9250.mag_valid) {
    imu_mpu9250.mag_valid = FALSE;
    _mag_handler();
  }
  if (imu_mpu9250.gyro_valid) {
    imu_mpu9250.gyro_valid = FALSE;
    _gyro_handler();
  }
}

#endif /* IMU_MPU9250_I2C_H */
