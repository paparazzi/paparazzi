/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/nav/nav_rotorcraft_base.h"
 * @author 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Basic navigation functions for Rotorcraft
 */

#ifndef NAV_ROTORCRAFT_BASE_H
#define NAV_ROTORCRAFT_BASE_H

#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/nav_base.h"

/** default nav_circle_radius in meters */
#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 5.f
#endif

#ifndef NAV_CLIMB_VSPEED
#define NAV_CLIMB_VSPEED 0.5f
#endif

#ifndef NAV_DESCEND_VSPEED
#define NAV_DESCEND_VSPEED -0.8f
#endif

/** minimum horizontal distance to waypoint to mark as arrived */
#ifndef ARRIVED_AT_WAYPOINT
#define ARRIVED_AT_WAYPOINT 3.0f
#endif

/** Maximum distance from HOME waypoint before going into failsafe mode */
#ifndef FAILSAFE_MODE_DISTANCE
#define FAILSAFE_MODE_DISTANCE (1.2*MAX_DIST_FROM_HOME)
#endif

#ifndef NAV_CARROT_DIST
#define NAV_CARROT_DIST 12
#endif

/** Basic Nav struct
 */
extern struct NavBase_t nav_rotorcraft_base;

extern void nav_rotorcraft_init(void);


/** Macros for circle nav
 */
#define NavCircleCount() nav_circle_get_count(&nav_rotorcraft_base.circle)
#define NavCircleQdr() nav_circle_qdr(&nav_rotorcraft_base.circle)

/** True if x (in degrees) is close to the current QDR (less than 10 degrees)
 */
#define NavQdrCloseTo(x) CloseDegAngles(x, NavCircleQdr())
#define NavCourseCloseTo(x) CloseDegAngles(x, DegOfRad(stateGetHorizontalSpeedDir_f()))


#endif

