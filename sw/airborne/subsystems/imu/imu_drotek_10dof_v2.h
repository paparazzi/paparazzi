/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_drotek_10dof_v2.h
 *
 * Driver for the Drotek 10DOF V2 IMU.
 * MPU6050 + HMC5883 + MS5611
 */

#ifndef IMU_DROTEK_10DOF_V2_H
#define IMU_DROTEK_10DOF_V2_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/mpu60x0_i2c.h"
#include "peripherals/hmc58xx.h"


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


struct ImuDrotek2 {
  volatile bool_t gyro_valid;
  volatile bool_t accel_valid;
  volatile bool_t mag_valid;
  struct Mpu60x0_I2c mpu;
  struct Hmc58xx hmc;
};

extern struct ImuDrotek2 imu_drotek2;

extern void imu_drotek2_event(void);
extern bool_t imu_drotek2_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu);


static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
  imu_drotek2_event();
  if (imu_drotek2.gyro_valid) {
    imu_drotek2.gyro_valid = FALSE;
    _gyro_handler();
  }
  if (imu_drotek2.accel_valid) {
    imu_drotek2.accel_valid = FALSE;
    _accel_handler();
  }
  if (imu_drotek2.mag_valid) {
    imu_drotek2.mag_valid = FALSE;
    _mag_handler();
  }
}

#endif /* IMU_DROTEK_10DOF_V2_H */
