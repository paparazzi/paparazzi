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
 * @file modules/imu/imu_mpu60x0_i2c.h
 * Driver for IMU with only MPU60x0 via I2C.
 */

#ifndef IMU_MPU60X0_I2C_H
#define IMU_MPU60X0_I2C_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/mpu60x0_i2c.h"

#ifndef IMU_MPU60X0_GYRO_RANGE
#define IMU_MPU60X0_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

#ifndef IMU_MPU60X0_ACCEL_RANGE
#define IMU_MPU60X0_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif

struct ImuMpu60x0 {
  struct Mpu60x0_I2c mpu;
};

extern struct ImuMpu60x0 imu_mpu_i2c;

extern void imu_mpu_i2c_init(void);
extern void imu_mpu_i2c_periodic(void);
extern void imu_mpu_i2c_event(void);

#endif /* IMU_MPU60X0_I2C_H */
