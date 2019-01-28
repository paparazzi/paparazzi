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

struct mt9v117_t {
  struct i2c_periph *i2c_periph;      ///< I2C peripheral used to communicate over
  struct i2c_transaction i2c_trans;   ///< I2C transaction for comminication with CMOS chip
};

extern struct mt9v117_t mt9v117;

void mt9v117_init(struct mt9v117_t *mt);

#endif /* MT9V117_H */
