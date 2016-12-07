/*
 * Copyright (C) Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/range_finder/teraranger_one.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Driver for the TeraRanger One range finder (I2C)
 */

#ifndef TERARANGER_ONE_H
#define TERARANGER_ONE_H

#include "std.h"

struct TeraRanger {
  uint16_t raw; ///< raw distance in mm
  float dist;   ///< scaled distance in m
  float offset; ///< offset in m
  bool data_available;
};

extern struct TeraRanger teraranger;

extern void teraranger_init(void);
extern void teraranger_periodic(void);
extern void teraranger_event(void);
extern void teraranger_downlink(void);

#endif

