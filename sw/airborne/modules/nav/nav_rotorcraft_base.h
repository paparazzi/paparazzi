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

#include "modules/nav/nav_base.h"

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

