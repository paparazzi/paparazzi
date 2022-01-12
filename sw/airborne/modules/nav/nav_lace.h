/*
 * Copyright (C) 2018-2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                         Titouan Verdu <titouan.verdu@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/nav/nav_lace.h
 *
 * Adaptive border pattern for cloud exploration
 * Can be used in mission mode with custom pattern and ID "LACE"
 *
 *
 * See:
 * Titouan Verdu, Gautier Hattenberger, Simon Lacroix. Flight patterns for clouds exploration with a fleet of UAVs. 2019 International Conference on Unmanned Aircraft Systems (ICUAS 2019), Jul 2019, Atlanta, United States.
 * https://hal-enac.archives-ouvertes.fr/hal-02137839
 */

#ifndef NAV_LACE_H
#define NAV_LACE_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

/** Init function called by modules init
 */
extern void nav_lace_init(void);

/** Initialized the exploration with a first target point inside the cloud
 *  Called from flight plan or with mission parameters
 */
extern void nav_lace_setup(float init_x, float init_y, float init_z, int turn, float desired_radius, float vx, float vy, float vz);

/** Navigation function
 *  Called by flight plan or mission run function
 *  @return true until pattern ends or fail
 */
extern bool nav_lace_run(void);

#endif

