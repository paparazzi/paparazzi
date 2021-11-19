/*
 * Copyright (C) 2010-2013 The Paparazzi Team
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
 * @file modules/imu/imu_gl1_defaults.h
 * Default sensitivity definitions for IMU GL1.
 */


#ifndef IMU_GL1_DEFAULTS_H
#define IMU_GL1_DEFAULTS_H

#include "generated/airframe.h"

/** default gyro sensitivy and neutral from the datasheet
 * L3G4200 has 8.75 LSB/(deg/s)
 * sens = 1/xxx * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/xxx * pi/180 * 4096 = ?????
 *
 * 250deg  = 114.28  = 0.625
 * 500deg  = 57.14   = 1.25
 * 2000deg = 14.28   = 5.006
 */
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 5.006
#define IMU_GYRO_P_SENS_NUM 2503
#define IMU_GYRO_P_SENS_DEN 500
#define IMU_GYRO_Q_SENS 5.006
#define IMU_GYRO_Q_SENS_NUM 2503
#define IMU_GYRO_Q_SENS_DEN 500
#define IMU_GYRO_R_SENS 5.006
#define IMU_GYRO_R_SENS_NUM 2503
#define IMU_GYRO_R_SENS_DEN 500
#endif

/** default accel sensitivy from the ADXL345 datasheet
 * sensitivity of x & y axes depends on supply voltage:
 *   - 256 LSB/g @ 2.5V
 *   - 265 LSB/g @ 3.3V
 * z sensitivity stays at 256 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 256 [LSB/g] * 2^INT32_ACCEL_FRAC
 * x/y sens = 9.81 / 265 * 1024 = 37.91
 * z sens   = 9.81 / 256 * 1024 = 39.24
 *
 * what about the offset at 3.3V?
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
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

#endif /* IMU_GL1_DEFAULTS_H */
