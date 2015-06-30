/*
 * Copyright (C) 2011 Paparazzi Team
 * Derived from Aspirin, NavGo and ppzuavimu drivers
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
 * @file boards/hbmini/imu_hbmini.c
 *
 * Driver for the IMU on the Hbmini board.
 *
 *  - Gyroscope: IDG 500 and ISZ 500, MAX1168
 *  - Accelerometer: Analog Devices ADXL320
 *  - Magnetometer: Honeywell HMC5883L
 */

#ifndef IMU_HBMINI_H
#define IMU_HBMINI_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/max1168.h"
#include "peripherals/hmc58xx.h"

// Default configuration
#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   -1
#define IMU_GYRO_Q_SIGN    1
#define IMU_GYRO_R_SIGN   -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN   1
#define IMU_ACCEL_Y_SIGN  -1
#define IMU_ACCEL_Z_SIGN  -1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    -1
#define IMU_MAG_Y_SIGN    -1
#define IMU_MAG_Z_SIGN    -1
#endif


struct ImuHbmini {
  struct Hmc58xx hmc;
};

extern struct ImuHbmini imu_hbmini;

/* Own Extra Functions */
extern void imu_hbmini_event(void);
extern void imu_hbmini_downlink_raw(void);

#define ImuEvent imu_hbmini_event

#endif
