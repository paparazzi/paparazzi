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
 * @file modules/nav/nav_survey_poly_osam.h
 *
 */

#ifndef NAV_SURVEY_POLY_OSAM_H
#define NAV_SURVEY_POLY_OSAM_H

#include "std.h"

extern uint8_t Poly_Size;
extern float Poly_Sweep;
/** Setup polygonsurvey */
/** Size is the number of waypoints used in poligon */
/** Sweep is the distance between scan lines */
/** Orientation is the angle of scan lines in relations to north. */
extern bool_t nav_survey_poly_osam_setup(uint8_t FirstWP, uint8_t Size, float Sweep, float Orientation);
/** start a polygonsurvey and compute the orientation angle from the line FirstWP-SecondWP */
/** if Size or Sweep = 0 you need to set it using telemetry settings */
/** if Size or Sweep != 0 it will be used in calculations */
/** To use the polysurvey dynamically you need to pass Size and Sweep as 0 and set it via telemetry. */
extern bool_t nav_survey_poly_osam_setup_towards(uint8_t FirstWP, uint8_t Size, float Sweep, int SecondWP);
extern bool_t nav_survey_poly_osam_run(void);
extern uint16_t PolySurveySweepNum;
extern uint16_t PolySurveySweepBackNum;

#endif
