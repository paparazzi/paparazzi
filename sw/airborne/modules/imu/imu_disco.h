/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/imu/imu_disco.h
 * Interface for the Disco magnetometer, accelerometer and gyroscope
 */


#ifndef IMU_DISCO_H
#define IMU_DISCO_H

#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/ak8963.h"
#include "peripherals/mpu60x0_i2c.h"

#ifndef DISCO_GYRO_RANGE
#define DISCO_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif

#ifndef DISCO_ACCEL_RANGE
#define DISCO_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS MPU60X0_GYRO_SENS[DISCO_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM MPU60X0_GYRO_SENS_FRAC[DISCO_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN MPU60X0_GYRO_SENS_FRAC[DISCO_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS MPU60X0_GYRO_SENS[DISCO_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM MPU60X0_GYRO_SENS_FRAC[DISCO_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN MPU60X0_GYRO_SENS_FRAC[DISCO_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS MPU60X0_GYRO_SENS[DISCO_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM MPU60X0_GYRO_SENS_FRAC[DISCO_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN MPU60X0_GYRO_SENS_FRAC[DISCO_GYRO_RANGE][1]
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS MPU60X0_ACCEL_SENS[DISCO_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[DISCO_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[DISCO_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS MPU60X0_ACCEL_SENS[DISCO_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[DISCO_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[DISCO_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS MPU60X0_ACCEL_SENS[DISCO_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[DISCO_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[DISCO_ACCEL_RANGE][1]
#endif

/** Everything that is in the disco IMU */
struct ImuDisco {
  struct Mpu60x0_I2c mpu;         ///< The MPU gyro/accel device
  struct Ak8963 ak;               ///< The AK8963 mag
};

extern struct ImuDisco imu_disco;

extern void imu_disco_init(void);
extern void imu_disco_periodic(void);
extern void imu_disco_event(void);

#endif /* IMU_DISCO_H */

