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
 * @file modules/imu/imu_mpu6000.h
 * Driver for IMU with only MPU6000 via SPI.
 */

#ifndef IMU_MPU6000_H
#define IMU_MPU6000_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/mpu60x0_spi.h"

#ifndef IMU_MPU_GYRO_RANGE
#define IMU_MPU_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

#ifndef IMU_MPU_ACCEL_RANGE
#define IMU_MPU_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS MPU60X0_GYRO_SENS[IMU_MPU_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM MPU60X0_GYRO_SENS_FRAC[IMU_MPU_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN MPU60X0_GYRO_SENS_FRAC[IMU_MPU_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS MPU60X0_GYRO_SENS[IMU_MPU_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM MPU60X0_GYRO_SENS_FRAC[IMU_MPU_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN MPU60X0_GYRO_SENS_FRAC[IMU_MPU_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS MPU60X0_GYRO_SENS[IMU_MPU_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM MPU60X0_GYRO_SENS_FRAC[IMU_MPU_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN MPU60X0_GYRO_SENS_FRAC[IMU_MPU_GYRO_RANGE][1]
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS MPU60X0_ACCEL_SENS[IMU_MPU_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[IMU_MPU_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[IMU_MPU_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS MPU60X0_ACCEL_SENS[IMU_MPU_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[IMU_MPU_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[IMU_MPU_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS MPU60X0_ACCEL_SENS[IMU_MPU_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[IMU_MPU_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[IMU_MPU_ACCEL_RANGE][1]
#endif


struct ImuMpu6000 {
  struct Mpu60x0_Spi mpu;
};

extern struct ImuMpu6000 imu_mpu_spi;

extern void imu_mpu_spi_init(void);
extern void imu_mpu_spi_periodic(void);
extern void imu_mpu_spi_event(void);

#endif /* IMU_MPU6000_H */
