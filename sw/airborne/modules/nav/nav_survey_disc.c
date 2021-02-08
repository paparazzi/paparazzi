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
 * @file modules/nav/nav_survey_disc.c
 *
 */

#include "modules/nav/nav_survey_disc.h"

#include "generated/airframe.h"
#include "state.h"
#include "std.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"
#include "math/pprz_algebra_float.h"

enum DiscSurveyStatus { UTURN, SEGMENT, DOWNWIND };

struct DiscSurvey {
  enum DiscSurveyStatus status;
  int8_t sign;
  struct FloatVect2 c;
  struct FloatVect2 c1;
  struct FloatVect2 c2;
};

static struct DiscSurvey disc_survey;


void nav_survey_disc_setup(float grid)
{
  nav_survey_shift = grid;
  disc_survey.status = DOWNWIND;
  disc_survey.sign = 1;
  disc_survey.c1.x = stateGetPositionEnu_f()->x;
  disc_survey.c1.y = stateGetPositionEnu_f()->y;
}

bool nav_survey_disc_run(uint8_t center_wp, float radius)
{
  struct FloatVect2 *wind = stateGetHorizontalWindspeed_f();
  float wind_dir = atan2(wind->x, wind->y) + M_PI;

  /** Not null even if wind_east=wind_north=0 */
  struct FloatVect2 upwind;
  upwind.x = cos(wind_dir);
  upwind.y = sin(wind_dir);

  float grid = nav_survey_shift / 2;

  switch (disc_survey.status) {
    case UTURN:
      nav_circle_XY(disc_survey.c.x, disc_survey.c.y, grid * disc_survey.sign);
      if (NavQdrCloseTo(DegOfRad(M_PI_2 - wind_dir))) {
        disc_survey.c1.x = stateGetPositionEnu_f()->x;
        disc_survey.c1.y = stateGetPositionEnu_f()->y;

        struct FloatVect2 dist;
        VECT2_DIFF(dist, disc_survey.c1, waypoints[center_wp]);
        float d = VECT2_DOT_PRODUCT(upwind, dist);
        if (d > radius) {
          disc_survey.status = DOWNWIND;
        } else {
          float w = sqrtf(radius * radius - d * d) - 1.5 * grid;

          struct FloatVect2 crosswind;
          crosswind.x = -upwind.y;
          crosswind.y = upwind.x;

          disc_survey.c2.x = waypoints[center_wp].x + d * upwind.x - w * disc_survey.sign * crosswind.x;
          disc_survey.c2.y = waypoints[center_wp].y + d * upwind.y - w * disc_survey.sign * crosswind.y;

          disc_survey.status = SEGMENT;
        }
        nav_init_stage();
      }
      break;

    case DOWNWIND:
      disc_survey.c2.x = waypoints[center_wp].x - upwind.x * radius;
      disc_survey.c2.y = waypoints[center_wp].y - upwind.y * radius;
      disc_survey.status = SEGMENT;
      /* No break; */
      /* fallthrough */

    case SEGMENT:
      nav_route_xy(disc_survey.c1.x, disc_survey.c1.y, disc_survey.c2.x, disc_survey.c2.y);
      if (nav_approaching_xy(disc_survey.c2.x, disc_survey.c2.y, disc_survey.c1.x, disc_survey.c1.y, CARROT)) {
        disc_survey.c.x = disc_survey.c2.x + grid * upwind.x;
        disc_survey.c.y = disc_survey.c2.y + grid * upwind.y;

        disc_survey.sign = -disc_survey.sign;
        disc_survey.status = UTURN;
        nav_init_stage();
      }
      break;
    default:
      break;
  }

  NavVerticalAutoThrottleMode(0.); /* No pitch */
  NavVerticalAltitudeMode(WaypointAlt(center_wp), 0.); /* No preclimb */

  return true;
}
