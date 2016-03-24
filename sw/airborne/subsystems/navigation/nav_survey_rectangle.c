/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
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
 * @file subsystems/navigation/nav_survey_rectangle.c
 *
 * Automatic survey of a rectangle for fixedwings.
 *
 * Rectangle is defined by two points, sweep can be south-north or west-east.
 */

#include "subsystems/navigation/nav_survey_rectangle.h"
#include "state.h"

static struct point survey_from;
static struct point survey_to;
static bool survey_uturn __attribute__((unused)) = false;
static survey_orientation_t survey_orientation = NS;

#define SurveyGoingNorth() ((survey_orientation == NS) && (survey_to.y > survey_from.y))
#define SurveyGoingSouth() ((survey_orientation == NS) && (survey_to.y < survey_from.y))
#define SurveyGoingEast() ((survey_orientation == WE) && (survey_to.x > survey_from.x))
#define SurveyGoingWest() ((survey_orientation == WE) && (survey_to.x < survey_from.x))

#include "generated/flight_plan.h"

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif


void nav_survey_rectangle_init(uint8_t wp1, uint8_t wp2, float grid, survey_orientation_t so)
{
  nav_survey_west = Min(WaypointX(wp1), WaypointX(wp2));
  nav_survey_east = Max(WaypointX(wp1), WaypointX(wp2));
  nav_survey_south = Min(WaypointY(wp1), WaypointY(wp2));
  nav_survey_north = Max(WaypointY(wp1), WaypointY(wp2));
  survey_orientation = so;

  if (survey_orientation == NS) {
    survey_from.x = survey_to.x = Min(Max(stateGetPositionEnu_f()->x, nav_survey_west + grid / 2.),
                                      nav_survey_east - grid / 2.);
    if (stateGetPositionEnu_f()->y > nav_survey_north || (stateGetPositionEnu_f()->y > nav_survey_south
                                                          && stateGetHorizontalSpeedDir_f() > M_PI / 2. && stateGetHorizontalSpeedDir_f() < 3 * M_PI / 2)) {
      survey_to.y = nav_survey_south;
      survey_from.y = nav_survey_north;
    } else {
      survey_from.y = nav_survey_south;
      survey_to.y = nav_survey_north;
    }
  } else { /* survey_orientation == WE */
    survey_from.y = survey_to.y = Min(Max(stateGetPositionEnu_f()->y, nav_survey_south + grid / 2.),
                                      nav_survey_north - grid / 2.);
    if (stateGetPositionEnu_f()->x > nav_survey_east || (stateGetPositionEnu_f()->x > nav_survey_west
        && stateGetHorizontalSpeedDir_f() > M_PI)) {
      survey_to.x = nav_survey_west;
      survey_from.x = nav_survey_east;
    } else {
      survey_from.x = nav_survey_west;
      survey_to.x = nav_survey_east;
    }
  }
  nav_survey_shift = grid;
  survey_uturn = false;
  LINE_START_FUNCTION;
}


void nav_survey_rectangle(uint8_t wp1, uint8_t wp2)
{
  static float survey_radius;

  nav_survey_active = true;

  nav_survey_west = Min(WaypointX(wp1), WaypointX(wp2));
  nav_survey_east = Max(WaypointX(wp1), WaypointX(wp2));
  nav_survey_south = Min(WaypointY(wp1), WaypointY(wp2));
  nav_survey_north = Max(WaypointY(wp1), WaypointY(wp2));

  /* Update the current segment from corners' coordinates*/
  if (SurveyGoingNorth()) {
    survey_to.y = nav_survey_north;
    survey_from.y = nav_survey_south;
  } else if (SurveyGoingSouth()) {
    survey_to.y = nav_survey_south;
    survey_from.y = nav_survey_north;
  } else if (SurveyGoingEast()) {
    survey_to.x = nav_survey_east;
    survey_from.x = nav_survey_west;
  } else if (SurveyGoingWest()) {
    survey_to.x = nav_survey_west;
    survey_from.x = nav_survey_east;
  }

  if (! survey_uturn) { /* S-N, N-S, W-E or E-W straight route */
    if ((stateGetPositionEnu_f()->y < nav_survey_north && SurveyGoingNorth()) ||
        (stateGetPositionEnu_f()->y > nav_survey_south && SurveyGoingSouth()) ||
        (stateGetPositionEnu_f()->x < nav_survey_east && SurveyGoingEast()) ||
        (stateGetPositionEnu_f()->x > nav_survey_west && SurveyGoingWest())) {
      /* Continue ... */
      nav_route_xy(survey_from.x, survey_from.y, survey_to.x, survey_to.y);
    } else {
      if (survey_orientation == NS) {
        /* North or South limit reached, prepare U-turn and next leg */
        float x0 = survey_from.x; /* Current longitude */
        if (x0 + nav_survey_shift < nav_survey_west || x0 + nav_survey_shift > nav_survey_east) {
          x0 += nav_survey_shift / 2;
          nav_survey_shift = -nav_survey_shift;
        }

        x0 = x0 + nav_survey_shift; /* Longitude of next leg */
        survey_from.x = survey_to.x = x0;

        /* Swap South and North extremities */
        float tmp = survey_from.y;
        survey_from.y = survey_to.y;
        survey_to.y = tmp;

        /** Do half a circle around WP 0 */
        waypoints[0].x = x0 - nav_survey_shift / 2.;
        waypoints[0].y = survey_from.y;

        /* Computes the right direction for the circle */
        survey_radius = nav_survey_shift / 2.;
        if (SurveyGoingNorth()) {
          survey_radius = -survey_radius;
        }
      } else { /* (survey_orientation == WE) */
        /* East or West limit reached, prepare U-turn and next leg */
        /* There is a y0 declared in math.h (for ARM) !!! */
        float my_y0 = survey_from.y; /* Current latitude */
        if (my_y0 + nav_survey_shift < nav_survey_south || my_y0 + nav_survey_shift > nav_survey_north) {
          my_y0 += nav_survey_shift / 2;
          nav_survey_shift = -nav_survey_shift;
        }

        my_y0 = my_y0 + nav_survey_shift; /* Longitude of next leg */
        survey_from.y = survey_to.y = my_y0;

        /* Swap West and East extremities */
        float tmp = survey_from.x;
        survey_from.x = survey_to.x;
        survey_to.x = tmp;

        /** Do half a circle around WP 0 */
        waypoints[0].x = survey_from.x;
        waypoints[0].y = my_y0 - nav_survey_shift / 2.;

        /* Computes the right direction for the circle */
        survey_radius = nav_survey_shift / 2.;
        if (SurveyGoingWest()) {
          survey_radius = -survey_radius;
        }
      }

      nav_in_segment = false;
      survey_uturn = true;
      LINE_STOP_FUNCTION;
    }
  } else { /* U-turn */
    if ((SurveyGoingNorth() && NavCourseCloseTo(0)) ||
        (SurveyGoingSouth() && NavCourseCloseTo(180)) ||
        (SurveyGoingEast() && NavCourseCloseTo(90)) ||
        (SurveyGoingWest() && NavCourseCloseTo(270))) {
      /* U-turn finished, back on a segment */
      survey_uturn = false;
      nav_in_circle = false;
      LINE_START_FUNCTION;
    } else {
      NavCircleWaypoint(0, survey_radius);
    }
  }
  NavVerticalAutoThrottleMode(0.); /* No pitch */
  NavVerticalAltitudeMode(WaypointAlt(wp1), 0.); /* No preclimb */
}
