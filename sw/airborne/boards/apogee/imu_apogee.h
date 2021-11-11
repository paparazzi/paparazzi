/*
 * Copyright (C) 2011 Gautier Hattenberger
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
 * @file boards/apogee/imu_apogee.h
 *
 * Driver for the IMU on the Apogee board.
 *
 * Invensense MPU-6050
 */

#ifndef IMU_APOGEE_H
#define IMU_APOGEE_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"
#if APOGEE_USE_MPU9150
#include "peripherals/ak8975.h"
#endif

#include "peripherals/mpu60x0_i2c.h"

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

#ifndef APOGEE_GYRO_RANGE
#define APOGEE_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif
#ifndef APOGEE_ACCEL_RANGE
#define APOGEE_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS MPU60X0_GYRO_SENS[APOGEE_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM MPU60X0_GYRO_SENS_FRAC[APOGEE_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN MPU60X0_GYRO_SENS_FRAC[APOGEE_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS MPU60X0_GYRO_SENS[APOGEE_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM MPU60X0_GYRO_SENS_FRAC[APOGEE_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN MPU60X0_GYRO_SENS_FRAC[APOGEE_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS MPU60X0_GYRO_SENS[APOGEE_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM MPU60X0_GYRO_SENS_FRAC[APOGEE_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN MPU60X0_GYRO_SENS_FRAC[APOGEE_GYRO_RANGE][1]
#endif
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif


// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS MPU60X0_ACCEL_SENS[APOGEE_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[APOGEE_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[APOGEE_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS MPU60X0_ACCEL_SENS[APOGEE_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[APOGEE_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[APOGEE_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS MPU60X0_ACCEL_SENS[APOGEE_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[APOGEE_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[APOGEE_ACCEL_RANGE][1]
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif

struct ImuApogee {
  struct Mpu60x0_I2c mpu;
#if APOGEE_USE_MPU9150
  struct Ak8975 ak;
#endif
};

extern struct ImuApogee imu_apogee;

extern void imu_apogee_init(void);
extern void imu_apogee_periodic(void);
extern void imu_apogee_event(void);
extern void imu_apogee_downlink_raw(void);

#endif // IMU_APOGEE_H
