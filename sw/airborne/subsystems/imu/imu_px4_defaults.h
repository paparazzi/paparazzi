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
 * @file subsystems/imu/imu_px4_defaults.h
 * Default sensitivity definitions for the Pixhawk IMU using the l3d20 gyro and lsm303dlc acc.
 */

#ifndef IMU_PX4_DEFAULTS_H
#define IMU_PX4_DEFAULTS_H

#include "generated/airframe.h"

/** default gyro sensitivy and neutral from the datasheet
 * L3GD20 has 70e-3 LSB/(deg/s) at 2000deg/s range
 * sens = 70e-3 * pi/180 * 2^INT32_RATE_FRAC
 * sens = (70e-3 / 180.0f) * pi * 4096
  */
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS_NUM 5004
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS_NUM 5004
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS_NUM 5004
#define IMU_GYRO_R_SENS_DEN 1000
#endif

/** default accel sensitivy from the datasheet
 * LSM303DLHC has 732 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 732 [LSB/g] * 2^INT32_ACCEL_FRAC
 * sens = 9.81 / 732 * 1024 = 13.72
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS

#define IMU_ACCEL_X_SENS 13.723
#define IMU_ACCEL_X_SENS_NUM 13723
#define IMU_ACCEL_X_SENS_DEN 1000
#define IMU_ACCEL_Y_SENS 13.723
#define IMU_ACCEL_Y_SENS_NUM 13723
#define IMU_ACCEL_Y_SENS_DEN 1000
#define IMU_ACCEL_Z_SENS 13.723
#define IMU_ACCEL_Z_SENS_NUM 13723
#define IMU_ACCEL_Z_SENS_DEN 1000
#endif

#endif /* IMU_PX4_DEFAULTS_H */
