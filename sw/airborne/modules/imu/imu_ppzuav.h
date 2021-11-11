/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/imu/imu_ppzuav.h
 *
 * Interface and defaults for the PPZUAV IMU.
 *
 * 9DoM IMU with ITG-3200, ADXL345 and HMC5843, all via I2C.
 */


#ifndef IMU_PPZUAV_H
#define IMU_PPZUAV_H

#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/itg3200.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/adxl345_i2c.h"

#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
/** default gyro sensitivy and neutral from the datasheet
 * ITG3200 has 14.375 LSB/(deg/s)
 * sens = 1/14.375 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/14.375 * pi/180 * 4096 = 4.973126
 */
#define IMU_GYRO_P_SENS 4.973
#define IMU_GYRO_P_SENS_NUM 4973
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.973
#define IMU_GYRO_Q_SENS_NUM 4973
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.973
#define IMU_GYRO_R_SENS_NUM 4973
#define IMU_GYRO_R_SENS_DEN 1000
#endif


#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
/** default accel sensitivy from the ADXL345 datasheet
 * sensitivity of x & y axes depends on supply voltage:
 *   - 256 LSB/g @ 2.5V
 *   - 265 LSB/g @ 3.3V
 * z sensitivity stays at 256 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 256 [LSB/g] * 2^INT32_ACCEL_FRAC
 * x/y sens = 9.81 / 265 * 1024 = 37.91
 * z sens   = 9.81 / 256 * 1024 = 39.24
 */
#define IMU_ACCEL_X_SENS 37.91
#define IMU_ACCEL_X_SENS_NUM 3791
#define IMU_ACCEL_X_SENS_DEN 100
#define IMU_ACCEL_Y_SENS 37.91
#define IMU_ACCEL_Y_SENS_NUM 3791
#define IMU_ACCEL_Y_SENS_DEN 100
#define IMU_ACCEL_Z_SENS 39.24
#define IMU_ACCEL_Z_SENS_NUM 3924
#define IMU_ACCEL_Z_SENS_DEN 100
#endif


struct ImuPpzuav {
  struct Adxl345_I2c acc_adxl;
  struct Itg3200 gyro_itg;
  struct Hmc58xx mag_hmc;
};

extern struct ImuPpzuav imu_ppzuav;

extern void imu_ppzuav_init(void);
extern void imu_ppzuav_periodic(void);
extern void imu_ppzuav_event(void);

#endif /* IMU_PPZUAV_H */
