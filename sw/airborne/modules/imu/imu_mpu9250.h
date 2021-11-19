/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file "modules/imu/imu_mpu9250.h"
 * @author Gautier Hattenberger
 *
 * Test module for the mpu9250
 */

#ifndef IMU_MPU9250_H
#define IMU_MPU9250_H

#include "peripherals/mpu9250_i2c.h"

extern struct Mpu9250_I2c mpu9250;

extern void imu_mpu9250_init(void);
extern void imu_mpu9250_periodic(void);
extern void imu_mpu9250_event(void);
extern void imu_mpu9250_report(void);

#endif

