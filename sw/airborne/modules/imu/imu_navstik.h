/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu_navstik.h
 * Interface for the Navstik magnetometer, accelerometer and gyroscope
 */


#ifndef IMU_NAVSTIK_H
#define IMU_NAVSTIK_H

#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/hmc58xx.h"
#include "peripherals/mpu60x0_i2c.h"

#ifndef NAVSTIK_GYRO_RANGE
#define NAVSTIK_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif

#ifndef NAVSTIK_ACCEL_RANGE
#define NAVSTIK_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS MPU60X0_GYRO_SENS[NAVSTIK_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM MPU60X0_GYRO_SENS_FRAC[NAVSTIK_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN MPU60X0_GYRO_SENS_FRAC[NAVSTIK_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS MPU60X0_GYRO_SENS[NAVSTIK_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM MPU60X0_GYRO_SENS_FRAC[NAVSTIK_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN MPU60X0_GYRO_SENS_FRAC[NAVSTIK_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS MPU60X0_GYRO_SENS[NAVSTIK_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM MPU60X0_GYRO_SENS_FRAC[NAVSTIK_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN MPU60X0_GYRO_SENS_FRAC[NAVSTIK_GYRO_RANGE][1]
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS MPU60X0_ACCEL_SENS[NAVSTIK_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[NAVSTIK_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[NAVSTIK_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS MPU60X0_ACCEL_SENS[NAVSTIK_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[NAVSTIK_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[NAVSTIK_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS MPU60X0_ACCEL_SENS[NAVSTIK_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM MPU60X0_ACCEL_SENS_FRAC[NAVSTIK_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN MPU60X0_ACCEL_SENS_FRAC[NAVSTIK_ACCEL_RANGE][1]
#endif


struct ImuNavstik {
  struct Mpu60x0_I2c mpu;
  struct Hmc58xx hmc;
};

extern struct ImuNavstik imu_navstik;

extern void imu_navstik_init(void);
extern void imu_navstik_periodic(void);
extern void imu_navstik_event(void);

#endif /* IMU_NAVSTIK_H */
