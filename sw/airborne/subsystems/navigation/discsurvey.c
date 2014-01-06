/*
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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
 * @file subsystems/navigation/discsurvey.c
 *
 */

#include "subsystems/navigation/discsurvey.h"

#include "generated/airframe.h"
#include "state.h"
#include "std.h"
#include "subsystems/nav.h"
#include "generated/flight_plan.h"

enum status { UTURN, SEGMENT, DOWNWIND };
static enum status status;
static int8_t sign;
static struct point c;
static struct point c1;
static struct point c2;

bool_t disc_survey_init( float grid ) {
  nav_survey_shift = grid;
  status = DOWNWIND;
  sign = 1;
  c1.x = stateGetPositionEnu_f()->x;
  c1.y = stateGetPositionEnu_f()->y;
  return FALSE;
}

bool_t disc_survey( uint8_t center, float radius) {
  struct FloatVect2* wind = stateGetHorizontalWindspeed_f();
  float wind_dir = atan2(wind->x, wind->y) + M_PI;

  /** Not null even if wind_east=wind_north=0 */
  float upwind_x = cos(wind_dir);
  float upwind_y = sin(wind_dir);

  float grid = nav_survey_shift / 2;

  switch (status) {
  case UTURN:
    nav_circle_XY(c.x, c.y, grid*sign);
    if (NavQdrCloseTo(DegOfRad(M_PI_2-wind_dir))) {
      c1.x = stateGetPositionEnu_f()->x;
      c1.y = stateGetPositionEnu_f()->y;

      float d = ScalarProduct(upwind_x, upwind_y, stateGetPositionEnu_f()->x-WaypointX(center), stateGetPositionEnu_f()->y-WaypointY(center));
      if (d > radius) {
        status = DOWNWIND;
      } else {
        float w = sqrt(radius*radius - d*d) - 1.5*grid;

        float crosswind_x = - upwind_y;
        float crosswind_y = upwind_x;

        c2.x = WaypointX(center)+d*upwind_x-w*sign*crosswind_x;
        c2.y = WaypointY(center)+d*upwind_y-w*sign*crosswind_y;

        status = SEGMENT;
      }
      nav_init_stage();
    }
    break;

  case DOWNWIND:
    c2.x = WaypointX(center) - upwind_x * radius;
    c2.y = WaypointY(center) - upwind_y * radius;
    status = SEGMENT;
    /* No break; */

  case SEGMENT:
    nav_route_xy(c1.x, c1.y, c2.x, c2.y);
    if (nav_approaching_xy(c2.x, c2.y, c1.x, c1.y, CARROT)) {
      c.x = c2.x + grid*upwind_x;
      c.y = c2.y + grid*upwind_y;

      sign = -sign;
      status = UTURN;
      nav_init_stage();
    }
    break;
  default:
    break;
  }

  NavVerticalAutoThrottleMode(0.); /* No pitch */
  NavVerticalAltitudeMode(WaypointAlt(center), 0.); /* No preclimb */

  return TRUE;
}
