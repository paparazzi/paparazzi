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
 *
 */

#ifndef IMU_UMARIM_H
#define IMU_UMARIM_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

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

extern volatile bool_t gyr_valid;
extern volatile bool_t acc_valid;

/* must be defined in order to be IMU code: declared in imu.h
extern void imu_impl_init(void);
extern void imu_periodic(void);
*/

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) { \
  imu_umarim_event();                                           \
  if (gyr_valid) {                                              \
    gyr_valid = FALSE;                                          \
    _gyro_handler();                                            \
  }                                                             \
  if (acc_valid) {                                              \
    acc_valid = FALSE;                                          \
    _accel_handler();                                           \
  }                                                             \
}

/* Own Extra Functions */
extern void imu_umarim_event( void );
extern void imu_umarim_downlink_raw( void );

#endif // PPZUAVIMU_H
