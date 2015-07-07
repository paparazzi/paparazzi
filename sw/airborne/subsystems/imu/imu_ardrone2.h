/*
 * Copyright (C) 2012-2013 Dino Hensen, Vincent van Hoek
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/imu/imu_ardrone2.h
 * IMU implementation for ardrone2.
 */

#ifndef IMU_ARDRONE2_H_
#define IMU_ARDRONE2_H_

#include "generated/airframe.h"
#include "navdata.h"


/** default gyro sensitivy and neutral from the datasheet
 * MPU with 2000 deg/s
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
 * 512 LSB/g
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 19.5
#define IMU_ACCEL_X_SENS_NUM 195
#define IMU_ACCEL_X_SENS_DEN 10
#define IMU_ACCEL_Y_SENS 19.5
#define IMU_ACCEL_Y_SENS_NUM 195
#define IMU_ACCEL_Y_SENS_DEN 10
#define IMU_ACCEL_Z_SENS 19.5
#define IMU_ACCEL_Z_SENS_NUM 195
#define IMU_ACCEL_Z_SENS_DEN 10
#endif

#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 2048
#define IMU_ACCEL_Y_NEUTRAL 2048
#define IMU_ACCEL_Z_NEUTRAL 2048
#endif

#if !defined IMU_MAG_X_SENS & !defined IMU_MAG_Y_SENS & !defined IMU_MAG_Z_SENS
#define IMU_MAG_X_SENS 16.0
#define IMU_MAG_X_SENS_NUM 16
#define IMU_MAG_X_SENS_DEN 1
#define IMU_MAG_Y_SENS 16.0
#define IMU_MAG_Y_SENS_NUM 16
#define IMU_MAG_Y_SENS_DEN 1
#define IMU_MAG_Z_SENS 16.0
#define IMU_MAG_Z_SENS_NUM 16
#define IMU_MAG_Z_SENS_DEN 1
#endif

/*
 * we include imh.h after the definitions of the neutrals
 */
#include "subsystems/imu.h"


#define ImuEvent navdata_update

#endif /* IMU_ARDRONE2_H_ */
