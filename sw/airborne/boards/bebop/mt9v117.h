/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
* @file boards/bebop/mt9v117.h
*
* Initialization and configuration of the MT9V117 CMOS Chip
*/

#ifndef MT9V117_H
#define MT9V117_H

#include "std.h"
#include "mcu_periph/i2c.h"

#ifndef MT9V117_TARGET_FPS
#define MT9V117_TARGET_FPS 0
#endif

// parameters for undistortion
#ifndef MT9V117_FOCAL_X
#define MT9V117_FOCAL_X 347.22f
#endif
#ifndef MT9V117_FOCAL_Y
#define MT9V117_FOCAL_Y 347.22f
#endif
#ifndef MT9V117_CENTER_X
#define MT9V117_CENTER_X 120.0f
#endif
#ifndef MT9V117_CENTER_Y
#define MT9V117_CENTER_Y 120.0f
#endif
#ifndef MT9V117_DHANE_K
#define MT9V117_DHANE_K 1.0f
#endif


struct mt9v117_t {
  struct i2c_periph *i2c_periph;      ///< I2C peripheral used to communicate over
  struct i2c_transaction i2c_trans;   ///< I2C transaction for comminication with CMOS chip
};

void mt9v117_init(struct mt9v117_t *mt);

#endif /* MT9V117_H */
