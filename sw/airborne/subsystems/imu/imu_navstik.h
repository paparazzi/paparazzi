/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file subsystems/imu/imu_navstik.h
 * Interface for the Navstik magnetometer, accelerometer and gyroscope
 */


#ifndef IMU_NAVSTIK_H
#define IMU_NAVSTIK_H

#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/hmc58xx.h"
#include "peripherals/mpu60x0_i2c.h"

/** default gyro sensitivy and neutral from the datasheet
 * MPU with 1000 deg/s has 32.8 LSB/(deg/s)
 * sens = 1/32.8 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/32.8 * pi/180 * 4096 = 2.17953
 */
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


struct ImuNavstik {
  volatile uint8_t accel_valid;
  volatile uint8_t gyro_valid;
  volatile uint8_t mag_valid;
  struct Mpu60x0_I2c mpu;
  struct Hmc58xx hmc;
};

extern struct ImuNavstik imu_navstik;
extern void imu_navstik_event(void);


static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
  imu_navstik_event();
  if (imu_navstik.gyro_valid) {
    imu_navstik.gyro_valid = FALSE;
    _gyro_handler();
  }
  if (imu_navstik.accel_valid) {
    imu_navstik.accel_valid = FALSE;
    _accel_handler();
  }
  if (imu_navstik.mag_valid) {
    imu_navstik.mag_valid = FALSE;
    _mag_handler();
  }
}

#endif /* IMU_NAVSTIK_H */
