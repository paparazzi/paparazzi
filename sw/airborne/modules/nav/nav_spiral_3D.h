/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/nav/nav_spiral_3D.h
 *
 * Fixedwing navigation in a 3D spiral.
 *
 */

#ifndef NAV_SPIRAL_3D_H
#define NAV_SPIRAL_3D_H

#include "std.h"
#include "math/pprz_algebra_float.h"

extern void nav_spiral_3D_init(void);

/** Run spiral 3D navigation
 */
extern bool nav_spiral_3D_run(void);

/** Initialize spiral 3D based on:
 *    - position X, Y
 *    - start and stop altitude
 *    - start and stop radius
 *    - speeds (horizontal and vertical)
 */
extern void nav_spiral_3D_setup(float center_x, float center_y,
                                float alt_start, float alt_stop,
                                float radius_start, float radius_stop,
                                float vx, float vy, float vz);

#endif // NAV_SPIRAL_3D_H

