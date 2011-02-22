/*
 * Copyright (C) 2010 The Paparazzi Team
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

extern volatile bool_t analog_imu_available;
extern int imu_overrun;

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) {   \
    if (analog_imu_available) {                         \
      analog_imu_available = FALSE;                     \
      _gyro_handler();                            \
      _accel_handler();                            \
    }                                                   \
    ImuMagEvent(_mag_handler);                          \
  }

#define ImuMagEvent(_mag_handler) {             \
    if (0) {                                    \
      _mag_handler();                           \
    }                                           \
  }




#endif /* IMU_ANALOG_H */
