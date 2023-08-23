/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/nav/dynamic_inspection_imav2023.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Compute and fly sequence of waypoints to maximize the score at IMAV2023, task 3
 */

#ifndef DYNAMIC_INSPECTION_IMAV2023_H
#define DYNAMIC_INSPECTION_IMAV2023_H

#include "std.h"

extern void dynamic_inspection_init(void);

/** Compute best trajectory in a sequence of points
 *  to maximize the score
 *
 *  @param[in] wp0 index of the first/last waypoint
 *  @param[in] nb number of waypoints (not including wp0, waypoints should be in order after wp0 in FP)
 *  @param[in] speed flight speed setpoint
 */
extern void dynamic_inspection_setup(uint8_t wp0, uint8_t nb, float speed);

/** Run best trajectory
 *  Should be called after setup
 */
extern bool dynamic_inspection_run(void);

#endif  // DYNAMIC_INSPECTION_IMAV2023_H

