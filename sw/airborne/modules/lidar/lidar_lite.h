/*
 * Copyright (C) 2016 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** @file modules/lidar/lidar_lite.h
 *  @brief driver for the Lidar-Lite i2c lidar
 *
 *  Note that the version 1 (silver label) seems to generate unexpected events
 *  on the i2c bus, such as misplaced start or stop or word reset (see I2C_ERRORS message).
 *  It seems to have no effect on other i2c devices (especially the IMU), but
 *  use with caution.
 *
 *  The newer versions function correctly.
 */
#ifndef LIDAR_LITE_I2C_H
#define LIDAR_LITE_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"

enum LidarLiteStatus {
	LIDAR_LITE_INIT_RANGING,
	LIDAR_LITE_REQ_READ,
	LIDAR_LITE_READ_DISTANCE,
	LIDAR_LITE_PARSE
};

struct LidarLite
{
  struct i2c_transaction trans;
  uint8_t addr;
  uint32_t distance_raw; // [cm]
  float distance; // [m]
  enum LidarLiteStatus status;
  bool update_agl;
  bool compensate_rotation;
};

extern struct LidarLite lidar_lite;

extern void lidar_lite_init(void);
extern void lidar_lite_event(void);
extern void lidar_lite_periodic(void);
extern void lidar_lite_downlink(void);

#endif /* LIDAR_LITE_I2C_H */

