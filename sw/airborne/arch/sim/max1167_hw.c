/*
 * Copyright (C) 2008  Antoine Drouin
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

#include "max1167_hw.h"

#include "subsystems/imu.h"

void max1167_hw_init(void) {}

void max1167_read(void) {}

void max1167_hw_feed_value(VEC *gyro, VEC *accel)
{

  max1167_values[0] = gyro->ve[AXIS_P];
  max1167_values[1] = gyro->ve[AXIS_Q];
  max1167_values[2] = gyro->ve[AXIS_R];

  buf_ax.sum =  accel->ve[AXIS_X];
  buf_ay.sum =  accel->ve[AXIS_Y];
  buf_az.sum =  accel->ve[AXIS_Z];

  max1167_status = MAX1167_DATA_AVAILABLE;

}
