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
 * @file modules/imu/imu_mpu6000_hmc5883.h
 * Driver for IMU with MPU6000 via SPI and HMC5883 via I2c.
 * E.g. for Aspirin 2.1
 */

#ifndef IMU_MPU6000_HMC5883_H
#define IMU_MPU6000_HMC5883_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/mpu60x0_spi.h"
#include "peripherals/hmc58xx.h"

#ifndef IMU_MPU_GYRO_RANGE
#define IMU_MPU_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

#ifndef IMU_MPU_ACCEL_RANGE
#define IMU_MPU_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif

struct ImuMpu6000Hmc5883 {
  struct Mpu60x0_Spi mpu;
  struct Hmc58xx hmc;
};

extern struct ImuMpu6000Hmc5883 imu_mpu_hmc;

extern void imu_mpu_hmc_init(void);
extern void imu_mpu_hmc_periodic(void);
extern void imu_mpu_hmc_event(void);

#endif /* IMU_MPU6000_HMC5883_H */
