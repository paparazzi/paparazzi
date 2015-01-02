/*
 * Copyright (C) 2011 Gautier Hattenberger
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
 * @file boards/apogee/imu_apogee.h
 *
 * Driver for the IMU on the Apogee board.
 *
 * Invensense MPU-6050
 */

#ifndef IMU_APOGEE_H
#define IMU_APOGEE_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/mpu60x0_i2c.h"

// Default configuration
#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif

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
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
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
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif

struct ImuApogee {
  volatile bool_t gyr_valid;
  volatile bool_t acc_valid;
  struct Mpu60x0_I2c mpu;
};

extern struct ImuApogee imu_apogee;


/* must be defined in order to be IMU code: declared in imu.h
extern void imu_impl_init(void);
extern void imu_periodic(void);
*/

/* Own Extra Functions */
extern void imu_apogee_event(void);
extern void imu_apogee_downlink_raw(void);


static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void),
                            void (* _mag_handler)(void) __attribute__((unused)))
{
  imu_apogee_event();
  if (imu_apogee.gyr_valid) {
    imu_apogee.gyr_valid = FALSE;
    _gyro_handler();
  }
  if (imu_apogee.acc_valid) {
    imu_apogee.acc_valid = FALSE;
    _accel_handler();
  }
}

#endif // IMU_APOGEE_H
