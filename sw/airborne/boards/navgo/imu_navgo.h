/*
 * Copyright (C) 2011 Gautier Hattenberger
 * Derived from Aspirin and ppzuavimu drivers
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
 * @file boards/navgo/imu_navgo.h
 *
 * Interface for the IMU on the NavGo board.
 *
 *  - Gyroscope: Invensense ITG-3200
 *  - Accelerometer: Analog Devices ADXL345
 *  - Magnetometer: Honeywell HMC5883L
 */

#ifndef IMU_NAVGO_H
#define IMU_NAVGO_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/itg3200.h"
#include "peripherals/adxl345_i2c.h"
#include "peripherals/hmc58xx.h"

// Default configuration
#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN    1
#define IMU_MAG_Z_SIGN    1
#endif


struct ImuNavgo {
  struct Itg3200 itg;
  struct Adxl345_I2c adxl;
  struct Hmc58xx hmc;
};

extern struct ImuNavgo imu_navgo;

extern void imu_navgo_init(void);
extern void imu_navgo_periodic(void);
extern void imu_navgo_event(void);
extern void imu_navgo_downlink_raw(void);

#endif // IMU_NAVGO_H
