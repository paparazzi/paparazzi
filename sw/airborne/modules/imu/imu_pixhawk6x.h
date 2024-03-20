/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu_pixhawk6x.h
 * Driver for the IMU's in the Pixhawk 6X autopilots.
 */

#ifndef IMU_PIXHAWK6X_H
#define IMU_PIXHAWK6X_H

#include "std.h"

extern void imu_pixhawk6x_init(void);
extern void imu_pixhawk6x_periodic(void);
extern void imu_pixhawk6x_event(void);

#endif /* IMU_PIXHAWK6X_H */
