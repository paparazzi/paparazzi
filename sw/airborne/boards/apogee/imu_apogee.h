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

/* Default range configurations */
#ifndef APOGEE_GYRO_RANGE
#define APOGEE_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif
#ifndef APOGEE_ACCEL_RANGE
#define APOGEE_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
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

#endif // IMU_APOGEE_H
