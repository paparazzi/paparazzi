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

struct ImuBmi088 {
  struct Bmi088_I2c bmi;
};

extern struct ImuBmi088 imu_bmi088;

extern void imu_bmi088_init(void);
extern void imu_bmi088_periodic(void);
extern void imu_bmi088_event(void);

#endif /* IMU_BMI088_I2C_H */
