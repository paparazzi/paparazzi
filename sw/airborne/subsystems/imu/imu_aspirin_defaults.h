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
 * @file subsystems/imu/imu_aspirin_defaults.h
 * Default sensitivity definitions for IMU Aspirin 1x.
 */


#ifndef IMU_ASPIRIN_DEFAULTS_H
#define IMU_ASPIRIN_DEFAULTS_H

#include "generated/airframe.h"

#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#ifdef IMU_ASPIRIN_VERSION_1_5
/** default gyro sensitivy and neutral from the datasheet
 * IMU-3000 has 16.4 LSB/(deg/s) at 2000deg/s range
 * sens = 1/16.4 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/16.4 * pi/180 * 4096 = 4.359066229
 */
#define IMU_GYRO_P_SENS 4.359
#define IMU_GYRO_P_SENS_NUM 4359
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.359
#define IMU_GYRO_Q_SENS_NUM 4359
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.359
#define IMU_GYRO_R_SENS_NUM 4359
#define IMU_GYRO_R_SENS_DEN 1000
#else
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
#endif // IMU_ASPIRIN_VERSION_1_5
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

#endif /* IMU_ASPIRIN_DEFAULTS_H */
