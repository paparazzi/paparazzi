/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/imu/imu_bmi088_i2c.h
 *
 * IMU driver for the BMI088 using I2C
 *
 */

#ifndef IMU_BMI088_I2C_H
#define IMU_BMI088_I2C_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/bmi088_i2c.h"


#ifndef IMU_BMI088_GYRO_RANGE
#define IMU_BMI088_GYRO_RANGE BMI088_GYRO_RANGE_1000
#endif

#ifndef IMU_BMI088_ACCEL_RANGE
#define IMU_BMI088_ACCEL_RANGE BMI088_ACCEL_RANGE_6G
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS BMI088_GYRO_SENS[IMU_BMI088_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM BMI088_GYRO_SENS_FRAC[IMU_BMI088_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN BMI088_GYRO_SENS_FRAC[IMU_BMI088_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS BMI088_GYRO_SENS[IMU_BMI088_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM BMI088_GYRO_SENS_FRAC[IMU_BMI088_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN BMI088_GYRO_SENS_FRAC[IMU_BMI088_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS BMI088_GYRO_SENS[IMU_BMI088_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM BMI088_GYRO_SENS_FRAC[IMU_BMI088_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN BMI088_GYRO_SENS_FRAC[IMU_BMI088_GYRO_RANGE][1]
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS BMI088_ACCEL_SENS[IMU_BMI088_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM BMI088_ACCEL_SENS_FRAC[IMU_BMI088_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN BMI088_ACCEL_SENS_FRAC[IMU_BMI088_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS BMI088_ACCEL_SENS[IMU_BMI088_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM BMI088_ACCEL_SENS_FRAC[IMU_BMI088_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN BMI088_ACCEL_SENS_FRAC[IMU_BMI088_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS BMI088_ACCEL_SENS[IMU_BMI088_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM BMI088_ACCEL_SENS_FRAC[IMU_BMI088_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN BMI088_ACCEL_SENS_FRAC[IMU_BMI088_ACCEL_RANGE][1]
#endif


struct ImuBmi088 {
  struct Bmi088_I2c bmi;
};

extern struct ImuBmi088 imu_bmi088;

extern void imu_bmi088_init(void);
extern void imu_bmi088_periodic(void);
extern void imu_bmi088_event(void);

#endif /* IMU_BMI088_I2C_H */
