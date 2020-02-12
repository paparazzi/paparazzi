/*
 * Copyright (C) 2008-2014 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/nav/nav_survey_poly_rotorcraft.h
 *
 */

#ifndef NAV_SURVEY_POLY_OSAM_H
#define NAV_SURVEY_POLY_OSAM_H

#include "std.h"

extern uint8_t Poly_Size;
extern float Poly_Distance;
extern float sweep_var;
extern uint16_t PolySurveySweepNum;
extern uint16_t PolySurveySweepBackNum;
extern bool Half_Sweep_Enabled;

/**
 * Setup polygon survey.
 * @param FirstWP      first waypoint/corner of the polygon
 * @param Size         number of waypoints/corners used to define the polygon
 * @param Sweep        distance between scan lines
 * @param Orientation  angle of scan lines in degrees (CCW, east)
 */
extern void nav_survey_poly_setup(uint8_t FirstWP, uint8_t Size, float Sweep, float Orientation);

/**
 * Setup "dynamic" polygon survey with sweep orientation towards a waypoint.
 * Computes the sweep orientation angle from the line FirstWP-SecondWP.
 * If you pass zero for Size and/or Sweep it will use the global Poly_Size and
 * Poly_Sweep variables respectively (which can be changed via telemetry/settings).
 * @param FirstWP   first waypoint/corner of the polygon
 * @param Size      number of waypoints/corners used to define the polygon,
 *                  if zero uses Poly_Size
 * @param Sweep     distance between scan lines, if zero uses Poly_Distance
 * @param SecondWP  second waypoint towards which the sweep orientation is computed
 */
extern void nav_survey_poly_setup_towards(uint8_t FirstWP, uint8_t Size, float Sweep, int SecondWP);

/** Run polygon survey */
extern bool nav_survey_poly_run(void);

#endif
