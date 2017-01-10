/*
 * Copyright (C) 2008-2015 The Paparazzi Team
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
 * @file modules/nav/nav_bungee_takeoff.h
 *
 * Takeoff functions for bungee takeoff.
 *
 * Run initialize function when the plane is on the bungee, the bungee is
 * fully extended and you are ready to launch the plane.
 * After initialized, the plane will follow a line drawn by the position
 * of the plane on initialization and the  position of the bungee (given in
 * the arguments).
 * Once the plane crosses the throttle line, which is perpendicular to the line
 * the plane is following, and intersects the position of the bungee (plus or
 * minus a fixed distance (BUNGEE_TAKEOFF_DISTANCE in airframe file) from
 * the bungee just in case the bungee doesn't release exactly above the bungee)
 * the prop will come on.
 * The plane will then continue to follow the line until it has reached a
 * specific height (defined in as BUNGEE_TAKEOFF_HEIGHT in airframe file) above
 * the bungee waypoint and airspeed (defined as BUNGEE_TAKEOFF_AIRSPEED in the
 * airframe file). The airspeed limit is only used if USE_AIRSPEED flag is
 * defined or set to true (and assuming the airspeed is then available).
 * It is also possible to specify the pitch angle (BUNGEE_TAKEOFF_PITCH) and
 * the throttle (BUNGEE_TAKEOFF_THROTTLE, between 0 and 1).
 *
 * @verbatim
 * <section name="BUNGEE" prefix="BUNGEE_TAKEOFF_">
 *   <define name="HEIGHT" value="30" unit="m"/>
 *   <define name="AIRSPEED" value="15" unit="m/s"/>
 *   <define name="DISTANCE" value="10" unit="m"/>
 *   <define name="MIN_SPEED" value="5" unit="m/s"/>
 *   <define name="PITCH" value="15." unit="deg"/>
 *   <define name="THROTTLE" value="1.0"/>
 * </section>
 * @endverbatim
 *
 *
 * from OSAM advanced navigation routines
 *
 */

#ifndef NAV_BUNGEE_TAKEOFF_H
#define NAV_BUNGEE_TAKEOFF_H

#include "std.h"

/** Initialization function
 *
 * called in the flight plan before the 'run' function
 *
 * @param[in] bungee_wp Waypoint ID correcponding to the bungee location
 */
extern void nav_bungee_takeoff_setup(uint8_t bungee_wp);

/** Bungee takeoff run function
 *
 * controls the different takeoff phases
 *
 * @return true until the takeoff procedure ends
 */
extern bool nav_bungee_takeoff_run(void);

#endif

