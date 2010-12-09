/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef IMU_ANALOG_H
#define IMU_ANALOG_H

#include "subsystems/imu.h"

#define NB_ANALOG_IMU_ADC 6

extern uint16_t analog_imu_values[NB_ANALOG_IMU_ADC];
extern volatile bool_t analog_imu_available;

#define ImuEvent(_gyro_accel_handler, _mag_handler) {		\
    if (ADSanalog_imu_available) {				\
      analog_imu_available = FALSE;				\
      imu.gyro_unscaled.p = analog_imu_values[0];		\
      imu.gyro_unscaled.q = analog_imu_values[1];		\
      imu.gyro_unscaled.r = analog_imu_values[2];		\
      imu.accel_unscaled.x = analog_imu_values[3];		\
      imu.accel_unscaled.y = analog_imu_values[4];		\
      imu.accel_unscaled.z = analog_imu_values[5];		\
      _gyro_accel_handler();					\
    }								\
    ImuMagEvent(_mag_handler);					\
  }

#define ImuMagEvent(_mag_handler) {					\
    if (false) {							\
      _mag_handler();							\
    }									\
  }




#endif /* IMU_CRISTA_H */
