/*
 * Copyright (C) Titouan Verdu
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
 * @file "modules/nav/nav_trinity.h"
 * @author Titouan Verdu
 * Adaptative trinity pattern for cloud exploration.
 */

#ifndef NAV_TRINITY_H
#define NAV_TRINITY_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

/** Init function called by modules init
 */
extern void nav_trinity_init(void);

/** Initialized the exploration with a first target point inside the cloud
 *  Called from flight plan or with mission parameters
 */
extern void nav_trinity_setup(float init_x, float init_y, float init_z, int turn, float desired_radius, float vx, float vy, float vz);

/** Navigation function
 *  Called by flight plan or mission run function
 *  @return true until pattern ends or fail
 */
extern bool nav_trinity_run(void);

#endif

