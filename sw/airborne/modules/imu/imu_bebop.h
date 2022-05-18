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
 * @file modules/imu/imu_bebop.h
 * Interface for the Bebop magnetometer, accelerometer and gyroscope
 */


#ifndef IMU_BEBOP_H
#define IMU_BEBOP_H

#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/ak8963.h"
#include "peripherals/mpu60x0_i2c.h"

#ifndef BEBOP_GYRO_RANGE
#define BEBOP_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif

#ifndef BEBOP_ACCEL_RANGE
#define BEBOP_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif

/** Everything that is in the bebop IMU */
struct ImuBebop {
  struct Mpu60x0_I2c mpu;         ///< The MPU gyro/accel device
  struct Ak8963 ak;               ///< The AK8963 mag
};

extern struct ImuBebop imu_bebop;

extern void imu_bebop_init(void);
extern void imu_bebop_periodic(void);
extern void imu_bebop_event(void);

#endif /* IMU_BEBOP_H */
