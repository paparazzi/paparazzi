/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
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
 * @file subsystems/imu/imu_altimu10.h
 *
 * Interface and defaults for the Pololu Altimu10 IMU.
 *
 * IMU with LSM6DS33, LIS3MDL and LPS25H, all via I2C.
 */


#ifndef IMU_ALTIMU10_H
#define IMU_ALTIMU10_H

#include "generated/airframe.h"

#include "peripherals/lsm6ds33_i2c.h"
#include "peripherals/lis3mdl_i2c.h"
#include "peripherals/lps25h_i2c.h"



#ifndef IMU_ALTIMU10_GYRO_RANGE
#define IMU_ALTIMU10_GYRO_RANGE LSM6_G_DEFAULT_FS
#endif

#ifndef IMU_ALTIMU10_ACCEL_RANGE
#define IMU_ALTIMU10_ACCEL_RANGE LSM6_XL_DEFAULT_FS
#endif

#ifndef IMU_ALTIMU10_MAG_RANGE
#define IMU_ALTIMU10_MAG_RANGE LIS3MDL_DEFAULT_FS
#endif

extern const float LSM6_GYRO_SENS[4];
extern const int32_t LSM6_GYRO_SENS_FRAC[4][2];

extern const float LSM6_ACCEL_SENS[4];
extern const int32_t LSM6_ACCEL_SENS_FRAC[4][2];

extern const float LIS3MDL_MAG_SENS[4];
extern const int32_t LIS3MDL_MAG_SENS_FRAC[4][2];


// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS LSM6_GYRO_SENS[IMU_ALTIMU10_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM LSM6_GYRO_SENS_FRAC[IMU_ALTIMU10_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN LSM6_GYRO_SENS_FRAC[IMU_ALTIMU10_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS LSM6_GYRO_SENS[IMU_ALTIMU10_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM LSM6_GYRO_SENS_FRAC[IMU_ALTIMU10_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN LSM6_GYRO_SENS_FRAC[IMU_ALTIMU10_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS LSM6_GYRO_SENS[IMU_ALTIMU10_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM LSM6_GYRO_SENS_FRAC[IMU_ALTIMU10_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN LSM6_GYRO_SENS_FRAC[IMU_ALTIMU10_GYRO_RANGE][1]
#endif

#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS LSM6_ACCEL_SENS[IMU_ALTIMU10_ACCEL_RANGE]
#define IMU_ACCEL_X_SENS_NUM LSM6_ACCEL_SENS_FRAC[IMU_ALTIMU10_ACCEL_RANGE][0]
#define IMU_ACCEL_X_SENS_DEN LSM6_ACCEL_SENS_FRAC[IMU_ALTIMU10_ACCEL_RANGE][1]
#define IMU_ACCEL_Y_SENS LSM6_ACCEL_SENS[IMU_ALTIMU10_ACCEL_RANGE]
#define IMU_ACCEL_Y_SENS_NUM LSM6_ACCEL_SENS_FRAC[IMU_ALTIMU10_ACCEL_RANGE][0]
#define IMU_ACCEL_Y_SENS_DEN LSM6_ACCEL_SENS_FRAC[IMU_ALTIMU10_ACCEL_RANGE][1]
#define IMU_ACCEL_Z_SENS LSM6_ACCEL_SENS[IMU_ALTIMU10_ACCEL_RANGE]
#define IMU_ACCEL_Z_SENS_NUM LSM6_ACCEL_SENS_FRAC[IMU_ALTIMU10_ACCEL_RANGE][0]
#define IMU_ACCEL_Z_SENS_DEN LSM6_ACCEL_SENS_FRAC[IMU_ALTIMU10_ACCEL_RANGE][1]
#endif

#if !defined IMU_MAG_X_SENS & !defined IMU_MAG_Y_SENS & !defined IMU_MAG_Z_SENS
#define IMU_MAG_X_SENS LIS3MDL_MAG_SENS[IMU_ALTIMU10_MAG_RANGE]
#define IMU_MAG_X_SENS_NUM LIS3MDL_MAG_SENS_FRAC[IMU_ALTIMU10_MAG_RANGE][0]
#define IMU_MAG_X_SENS_DEN LIS3MDL_MAG_SENS_FRAC[IMU_ALTIMU10_MAG_RANGE][1]
#define IMU_MAG_Y_SENS LIS3MDL_MAG_SENS[IMU_ALTIMU10_MAG_RANGE]
#define IMU_MAG_Y_SENS_NUM LIS3MDL_MAG_SENS_FRAC[IMU_ALTIMU10_MAG_RANGE][0]
#define IMU_MAG_Y_SENS_DEN LIS3MDL_MAG_SENS_FRAC[IMU_ALTIMU10_MAG_RANGE][1]
#define IMU_MAG_Z_SENS LIS3MDL_MAG_SENS[IMU_ALTIMU10_MAG_RANGE]
#define IMU_MAG_Z_SENS_NUM LIS3MDL_MAG_SENS_FRAC[IMU_ALTIMU10_MAG_RANGE][0]
#define IMU_MAG_Z_SENS_DEN LIS3MDL_MAG_SENS_FRAC[IMU_ALTIMU10_MAG_RANGE][1]
#endif

#include "subsystems/imu.h"

struct ImuAltimu10 {
  struct Lsm6_I2c acc_g_lsm6;
  struct Lis3mdl_I2c mag_lis3mdl;
  struct Lps25h_I2c baro_lps25h;
};

extern struct ImuAltimu10 imu_altimu10;

extern void imu_altimu10_init(void);
extern void imu_altimu10_periodic(void);
extern void imu_altimu10_event(void);

#endif /* IMU_PPZUAV_H */
