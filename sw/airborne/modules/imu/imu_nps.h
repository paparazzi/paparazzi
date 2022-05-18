/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com
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

#ifndef IMU_NPS_H
#define IMU_NPS_H

#include "modules/imu/imu.h"

#include "generated/airframe.h"

struct ImuNps {
  uint8_t mag_available;
  uint8_t accel_available;
  uint8_t gyro_available;

  struct Int32Rates gyro;
  struct Int32Vect3 accel;
  struct Int32Vect3 mag;
};

extern struct ImuNps imu_nps;

extern void imu_feed_gyro_accel(void);
extern void imu_feed_mag(void);

extern void imu_nps_init(void);
extern void imu_nps_event(void);

#endif /* IMU_NPS_H */
