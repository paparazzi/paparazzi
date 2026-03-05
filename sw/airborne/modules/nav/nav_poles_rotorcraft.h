/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/nav/nav_poles_rotorcraft.h
 *
 * Turn around 2 points, with possible margins
 * Can be used in mission mode
 *
 */

#ifndef NAV_POLES_ROTORCRAFT_H
#define NAV_POLES_ROTORCRAFT_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

extern uint8_t nav_poles_count;

/** Global init
 */
extern void nav_poles_init(void);

/** Init poles from flight plan waypoints
 *
 * @param[in] wp1 waypoint ID
 * @param[in] wp2 waypoint ID
 * @param[in] height flight height above ref point
 * @param[in] radius turn radius in meters, CW is >0, CCW otherwise
 * @param[in] margin margin factor: 0 -> overfly points, 1 -> turn at 'radius' distance
 * @return true if init valid
 */
extern bool nav_poles_setup_wp(uint8_t wp1, uint8_t wp2, float height,
    float radius, float margin, int8_t nb_laps);

/** Init poles from waypoints coordinates in LLA format
 *
 * @param[in] wp1 waypoint LLA position
 * @param[in] wp2 waypoint LLA position
 * @param[in] height flight height above ref point
 * @param[in] radius turn radius in meters, CW is >0, CCW otherwise
 * @param[in] margin margin factor: 0 -> overfly points, 1 -> turn at 'radius' distance
 * @param[in] nb_laps number of laps (<0 for no lap limit)
 * @return true if init valid
 */
extern bool nav_poles_setup_lla(struct LlaCoor_f *lla1, struct LlaCoor_f *lla2, float height,
    float radius, float margin, int8_t nb_laps);

extern bool nav_poles_run(void);

#endif

