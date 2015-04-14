/*
 * Copyright (C) 2013 Sergey Krukowski <softsr@yahoo.de>
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
 * @file boards/krooz/imu_krooz.h
 *
 * Driver for the IMU on the KroozSD board.
 *
 * Invensense MPU-6050
 */

#ifndef IMU_KROOZ_H
#define IMU_KROOZ_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/mpu60x0_i2c.h"
#include "peripherals/hmc58xx.h"

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
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN    1
#define IMU_MAG_Z_SIGN    1
#endif

/** default gyro sensitivy and neutral from the datasheet
 * MPU with 250 deg/s has 131.072 LSB/(deg/s)
 * sens = 1/131.072 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/131.072 * pi/180 * 4096 = 0.5454
 I*/
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
// FIXME
#define IMU_GYRO_P_SENS 0.5454
#define IMU_GYRO_P_SENS_NUM 2727
#define IMU_GYRO_P_SENS_DEN 5000
#define IMU_GYRO_Q_SENS 0.5454
#define IMU_GYRO_Q_SENS_NUM 2727
#define IMU_GYRO_Q_SENS_DEN 5000
#define IMU_GYRO_R_SENS 0.5454
#define IMU_GYRO_R_SENS_NUM 2727
#define IMU_GYRO_R_SENS_DEN 5000
#endif
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif


/** default accel sensitivy from the datasheet
 * MPU with 2g has 16384 LSB/g
 * sens = 9.81 [m/s^2] / 16384 [LSB/g] * 2^INT32_ACCEL_FRAC = 0.6131
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
// FIXME
#define IMU_ACCEL_X_SENS 0.6131
#define IMU_ACCEL_X_SENS_NUM 6131
#define IMU_ACCEL_X_SENS_DEN 10000
#define IMU_ACCEL_Y_SENS 0.6131
#define IMU_ACCEL_Y_SENS_NUM 6131
#define IMU_ACCEL_Y_SENS_DEN 10000
#define IMU_ACCEL_Z_SENS 0.6131
#define IMU_ACCEL_Z_SENS_NUM 6131
#define IMU_ACCEL_Z_SENS_DEN 10000
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif

#ifndef IMU_KROOZ_ACCEL_AVG_FILTER
#define IMU_KROOZ_ACCEL_AVG_FILTER      15
#endif

struct ImuKrooz {
  volatile bool_t mpu_eoc;
  volatile bool_t hmc_eoc;
  struct Mpu60x0_I2c mpu;
  struct Hmc58xx hmc;
  struct Int32Rates rates_sum;
  struct Int32Vect3 accel_sum;
  volatile uint8_t  meas_nb;
  struct Int32Vect3 accel_filtered;
  int32_t temperature;
};

extern struct ImuKrooz imu_krooz;


/* must be defined in order to be IMU code: declared in imu.h
extern void imu_impl_init(void);
extern void imu_periodic(void);
*/

/* Own Extra Functions */
extern void imu_krooz_event(void);
extern void imu_krooz_downlink_raw(void);

#define ImuEvent imu_krooz_event

#endif // IMU_KROOZ_H
