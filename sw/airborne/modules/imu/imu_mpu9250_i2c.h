/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/imu/imu_mpu9250_i2c.h
 *
 * IMU driver for the MPU9250 using I2C
 *
 */

#ifndef IMU_MPU9250_I2C_H
#define IMU_MPU9250_I2C_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/mpu9250_i2c.h"


#ifndef IMU_MPU9250_GYRO_RANGE
#define IMU_MPU9250_GYRO_RANGE MPU9250_GYRO_RANGE_1000
#endif

#ifndef IMU_MPU9250_ACCEL_RANGE
#define IMU_MPU9250_ACCEL_RANGE MPU9250_ACCEL_RANGE_8G
#endif

struct ImuMpu9250 {
  struct Mpu9250_I2c mpu;
};

extern struct ImuMpu9250 imu_mpu9250;

extern void imu_mpu9250_init(void);
extern void imu_mpu9250_periodic(void);
extern void imu_mpu9250_event(void);

#endif /* IMU_MPU9250_I2C_H */
