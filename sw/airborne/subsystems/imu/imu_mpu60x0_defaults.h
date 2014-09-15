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
 * @file subsystems/imu/imu_mpu60x0_defaults.h
 * Default sensitivity definitions for an IMU using the MPU60x0.
 */

#ifndef IMU_MPU60X0_DEFAULTS_H
#define IMU_MPU60X0_DEFAULTS_H

#include "generated/airframe.h"

/** default gyro sensitivy and neutral from the datasheet
 * MPU60X0 has 16.4 LSB/(deg/s) at 2000deg/s range
 * sens = 1/16.4 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/16.4 * pi/180 * 4096 = 4.359066229
 */
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 4.359
#define IMU_GYRO_P_SENS_NUM 4359
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.359
#define IMU_GYRO_Q_SENS_NUM 4359
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.359
#define IMU_GYRO_R_SENS_NUM 4359
#define IMU_GYRO_R_SENS_DEN 1000
#endif

/** default accel sensitivy from the datasheet
 * MPU60X0 has 2048 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 2048 [LSB/g] * 2^INT32_ACCEL_FRAC
 * sens = 9.81 / 2048 * 1024 = 4.905
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 4.905
#define IMU_ACCEL_X_SENS_NUM 4905
#define IMU_ACCEL_X_SENS_DEN 1000
#define IMU_ACCEL_Y_SENS 4.905
#define IMU_ACCEL_Y_SENS_NUM 4905
#define IMU_ACCEL_Y_SENS_DEN 1000
#define IMU_ACCEL_Z_SENS 4.905
#define IMU_ACCEL_Z_SENS_NUM 4905
#define IMU_ACCEL_Z_SENS_DEN 1000
#endif

#endif /* IMU_MPU60X0_DEFAULTS_H */
