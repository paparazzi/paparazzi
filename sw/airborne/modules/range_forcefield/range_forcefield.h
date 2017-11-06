/*
 * Copyright (C) 2017 K. N. McGuire
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
/*
 * @file "modules/range_forcefield/range_forcefield.h"
 * @author K. N. McGuire
 * This module generates a forcefield based on range sensor measurements the use of single point range sensors.
 */

#ifndef RANGE_FORCEFIELD_H
#define RANGE_FORCEFIELD_H

#include <std.h>
#include "math/pprz_algebra_float.h"

struct range_forcefield_param_t {
  float  inner_limit;
  float  outer_limit;
  float  min_vel;
  float  max_vel;
};

extern struct range_forcefield_param_t range_forcefield_param;

extern void range_forcefield_init(void);
extern void range_forcefield_periodic(void);
extern void range_forcefield_update(float range, struct FloatEulers *body_to_sensor_eulers);

#endif
