/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu_cube.h
 * Driver for the IMU's in the Cube autopilots.
 */

#ifndef IMU_CUBE_H
#define IMU_CUBE_H

#include "std.h"

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 1
#define IMU_GYRO_P_SENS_NUM 1
#define IMU_GYRO_P_SENS_DEN 1
#define IMU_GYRO_Q_SENS 1
#define IMU_GYRO_Q_SENS_NUM 1
#define IMU_GYRO_Q_SENS_DEN 1
#define IMU_GYRO_R_SENS 1
#define IMU_GYRO_R_SENS_NUM 1
#define IMU_GYRO_R_SENS_DEN 1
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 1
#define IMU_ACCEL_X_SENS_NUM 1
#define IMU_ACCEL_X_SENS_DEN 1
#define IMU_ACCEL_Y_SENS 1
#define IMU_ACCEL_Y_SENS_NUM 1
#define IMU_ACCEL_Y_SENS_DEN 1
#define IMU_ACCEL_Z_SENS 1
#define IMU_ACCEL_Z_SENS_NUM 1
#define IMU_ACCEL_Z_SENS_DEN 1
#endif

extern void imu_cube_init(void);
extern void imu_cube_periodic(void);
extern void imu_cube_event(void);

#endif /* IMU_CUBE_H */
