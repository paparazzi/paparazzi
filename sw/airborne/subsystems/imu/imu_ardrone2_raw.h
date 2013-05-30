/*
 * Copyright (C) 2012-2013 Dino Hensen, Vincent van Hoek
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/imu/imu_ardrone2_raw.h
 * IMU implementation for ardrone2-raw.
 */

#ifndef IMU_ARDRONE2_RAW_H_
#define IMU_ARDRONE2_RAW_H_

#include "subsystems/imu.h"
#include "generated/airframe.h"
#include "navdata.h"

int imu_data_available;

static inline void imu_ardrone2_event ( void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
  if (imu_data_available) {
    imu_data_available = FALSE;
    _gyro_handler();
    _accel_handler();
    _mag_handler();
  }
}

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) {  \
    imu_ardrone2_event(_gyro_handler, _accel_handler, _mag_handler); \
}

#endif /* IMU_ARDRONE2_RAW_H_ */
