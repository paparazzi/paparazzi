/*
 * Copyright (C) 2011-2013  The Paparazzi Team
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
 * @file modules/nav/nav_survey_polygon.c
 *
 * Advanced polygon survey for fixedwings from Uni Stuttgart.
 *
 */

#include "nav_survey_polygon.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

struct SurveyPolyAdv survey;

static void nav_points(struct FloatVect2 start, struct FloatVect2 end)
{
  nav_route_xy(start.x, start.y, end.x, end.y);
}

/**
 * intercept two lines and give back the point of intersection
 * @return         FALSE if no intersection can be found or intersection does not lie between points a and b
 * else TRUE
 * @param p               returns intersection
 * @param x, y            first line is defined by point x and y (goes through this points)
 * @param a1, a2, b1, b2  second line by coordinates a1/a2, b1/b2
 */
static bool intercept_two_lines(struct FloatVect2 *p, struct FloatVect2 x, struct FloatVect2 y, float a1, float a2,
                                  float b1, float b2)
{
  float divider, fac;

  divider = (((b2 - a2) * (y.x - x.x)) + ((x.y - y.y) * (b1 - a1)));
  if (divider == 0) { return false; }
  fac = ((y.x * (x.y - a2)) + (x.x * (a2 - y.y)) + (a1 * (y.y - x.y))) / divider;
  if (fac > 1.0) { return false; }
  if (fac < 0.0) { return false; }

  p->x = a1 + fac * (b1 - a1);
  p->y = a2 + fac * (b2 - a2);

  return true;
}

/**
 *  intersects a line with the polygon and gives back the two intersection points
 *  @return        TRUE if two intersection can be found, else FALSE
 *  @param x, y     intersection points
 *  @param a, b     define the line to intersection
 */
static bool get_two_intersects(struct FloatVect2 *x, struct FloatVect2 *y, struct FloatVect2 a, struct FloatVect2 b)
{
  int i, count = 0;
  struct FloatVect2 tmp;

  for (i = 0; i < survey.poly_count - 1; i++)
    if (intercept_two_lines(&tmp, a, b, waypoints[survey.poly_first + i].x, waypoints[survey.poly_first + i].y,
                            waypoints[survey.poly_first + i + 1].x, waypoints[survey.poly_first + i + 1].y)) {
      if (count == 0) {
        *x = tmp;
        count++;
      } else {
        *y = tmp;
        count++;
        break;
      }
    }

  //wrapover first,last polygon waypoint
  if (count == 1
      && intercept_two_lines(&tmp, a, b, waypoints[survey.poly_first + survey.poly_count - 1].x,
                             waypoints[survey.poly_first + survey.poly_count - 1].y, waypoints[survey.poly_first].x,
                             waypoints[survey.poly_first].y)) {
    *y = tmp;
    count++;
  }

  if (count != 2) {
    return false;
  }

  //change points
  if (fabs(survey.dir_vec.x) > fabs(survey.dir_vec.y)) {
    if ((y->x - x->x) / survey.dir_vec.x < 0.0) {
      tmp = *x;
      *x = *y;
      *y = tmp;
    }
  } else if ((y->y - x->y) / survey.dir_vec.y < 0.0) {
    tmp = *x;
    *x = *y;
    *y = tmp;
  }

  return true;
}

/**
 *  initializes the variables needed for the survey to start
 *  @param first_wp      the first Waypoint of the polygon
 *  @param size          the number of points that make up the polygon
 *  @param angle         angle in which to do the flyovers
 *  @param sweep_width   distance between the sweeps
 *  @param shot_dist     distance between the shots
 *  @param min_rad       minimal radius when navigating
 *  @param altitude      the altitude that must be reached before the flyover starts
 **/
bool nav_survey_polygon_setup(uint8_t first_wp, uint8_t size, float angle, float sweep_width, float shot_dist,
                                float min_rad, float altitude)
{
  int i;
  struct FloatVect2 small, sweep;
  float divider, angle_rad = angle / 180.0 * M_PI;

  if (angle < 0.0) { angle += 360.0; }
  if (angle >= 360.0) { angle -= 360.0; }

  survey.poly_first = first_wp;
  survey.poly_count = size;

  survey.psa_sweep_width = sweep_width;
  survey.psa_min_rad = min_rad;
  survey.psa_shot_dist = shot_dist;
  survey.psa_altitude = altitude;

  survey.segment_angle = angle;
  survey.return_angle = angle + 180;
  if (survey.return_angle > 359) { survey.return_angle -= 360; }

  if (angle <= 45.0 || angle >= 315.0) {
    //north
    survey.dir_vec.y = 1.0;
    survey.dir_vec.x = 1.0 * tanf(angle_rad);
    sweep.x = 1.0;
    sweep.y = - survey.dir_vec.x / survey.dir_vec.y;
  } else if (angle <= 135.0) {
    //east
    survey.dir_vec.x = 1.0;
    survey.dir_vec.y = 1.0 / tanf(angle_rad);
    sweep.y = - 1.0;
    sweep.x = survey.dir_vec.y / survey.dir_vec.x;
  } else if (angle <= 225.0) {
    //south
    survey.dir_vec.y = -1.0;
    survey.dir_vec.x = -1.0 * tanf(angle_rad);
    sweep.x = -1.0;
    sweep.y = survey.dir_vec.x / survey.dir_vec.y;
  } else {
    //west
    survey.dir_vec.x = -1.0;
    survey.dir_vec.y = -1.0 / tanf(angle_rad);
    sweep.y = 1.0;
    sweep.x = - survey.dir_vec.y / survey.dir_vec.x;
  }

  //normalize
  FLOAT_VECT2_NORMALIZE(sweep);

  VECT2_SMUL(survey.rad_vec, sweep, survey.psa_min_rad);
  VECT2_SMUL(survey.sweep_vec, sweep, survey.psa_sweep_width);

  //begin at leftmost position (relative to survey.dir_vec)
  VECT2_COPY(small, waypoints[survey.poly_first]);

  divider = (survey.sweep_vec.y * survey.dir_vec.x) - (survey.sweep_vec.x * survey.dir_vec.y);

  //calculate the leftmost point if one sees the dir vec as going "up" and the sweep vec as going right
  if (divider < 0.0) {
    for (i = 1; i < survey.poly_count; i++)
      if ((survey.dir_vec.x * (waypoints[survey.poly_first + i].y - small.y)) + (survey.dir_vec.y *
          (small.x - waypoints[survey.poly_first + i].x)) > 0.0) {
        VECT2_COPY(small, waypoints[survey.poly_first + i]);
      }
  } else
    for (i = 1; i < survey.poly_count; i++)
      if ((survey.dir_vec.x * (waypoints[survey.poly_first + i].y - small.y)) + (survey.dir_vec.y *
          (small.x - waypoints[survey.poly_first + i].x)) > 0.0) {
        VECT2_COPY(small, waypoints[survey.poly_first + i]);
      }

  //calculate the line the defines the first flyover
  survey.seg_start.x = small.x + 0.5 * survey.sweep_vec.x;
  survey.seg_start.y = small.y + 0.5 * survey.sweep_vec.y;
  VECT2_SUM(survey.seg_end, survey.seg_start, survey.dir_vec);

  if (!get_two_intersects(&survey.seg_start, &survey.seg_end, survey.seg_start, survey.seg_end)) {
    survey.stage = ERR;
    return false;
  }

  //center of the entry circle
  VECT2_DIFF(survey.entry_center, survey.seg_start, survey.rad_vec);

  //fast climbing to desired altitude
  NavVerticalAutoThrottleMode(0.0);
  NavVerticalAltitudeMode(survey.psa_altitude, 0.0);

  survey.stage = ENTRY;

  return false;
}

/**
 * main navigation routine. This is called periodically evaluates the current
 * Position and stage and navigates accordingly.
 * @returns True until the survey is finished
 */
bool nav_survey_polygon_run(void)
{
  NavVerticalAutoThrottleMode(0.0);
  NavVerticalAltitudeMode(survey.psa_altitude, 0.0);

  //entry circle around entry-center until the desired altitude is reached
  if (survey.stage == ENTRY) {
    nav_circle_XY(survey.entry_center.x, survey.entry_center.y, -survey.psa_min_rad);
    if (NavCourseCloseTo(survey.segment_angle)
        && nav_approaching_xy(survey.seg_start.x, survey.seg_start.y, last_x, last_y, CARROT)
        && fabs(stateGetPositionUtm_f()->alt - survey.psa_altitude) <= 20) {
      survey.stage = SEG;
      nav_init_stage();
#ifdef DIGITAL_CAM
      dc_survey(survey.psa_shot_dist, survey.seg_start.x - survey.dir_vec.x * survey.psa_shot_dist * 0.5,
                survey.seg_start.y - survey.dir_vec.y * survey.psa_shot_dist * 0.5);
#endif
    }
  }
  //fly the segment until seg_end is reached
  if (survey.stage == SEG) {
    nav_points(survey.seg_start, survey.seg_end);
    //calculate all needed points for the next flyover
    if (nav_approaching_xy(survey.seg_end.x, survey.seg_end.y, survey.seg_start.x, survey.seg_start.y, 0)) {
#ifdef DIGITAL_CAM
      dc_stop();
#endif
      VECT2_DIFF(survey.seg_center1, survey.seg_end, survey.rad_vec);
      survey.ret_start.x = survey.seg_end.x - 2 * survey.rad_vec.x;
      survey.ret_start.y = survey.seg_end.y - 2 * survey.rad_vec.y;

      //if we get no intersection the survey is finished
      static struct FloatVect2 sum_start_sweep;
      static struct FloatVect2 sum_end_sweep;
      VECT2_SUM(sum_start_sweep, survey.seg_start, survey.sweep_vec);
      VECT2_SUM(sum_end_sweep, survey.seg_end, survey.sweep_vec);
      if (!get_two_intersects(&survey.seg_start, &survey.seg_end, sum_start_sweep, sum_end_sweep)) {
        return false;
      }

      survey.ret_end.x = survey.seg_start.x - survey.sweep_vec.x - 2 * survey.rad_vec.x;
      survey.ret_end.y = survey.seg_start.y - survey.sweep_vec.y - 2 * survey.rad_vec.y;

      survey.seg_center2.x = survey.seg_start.x - 0.5 * (2.0 * survey.rad_vec.x + survey.sweep_vec.x);
      survey.seg_center2.y = survey.seg_start.y - 0.5 * (2.0 * survey.rad_vec.y + survey.sweep_vec.y);

      survey.stage = TURN1;
      nav_init_stage();
    }
  }
  //turn from stage to return
  else if (survey.stage == TURN1) {
    nav_circle_XY(survey.seg_center1.x, survey.seg_center1.y, -survey.psa_min_rad);
    if (NavCourseCloseTo(survey.return_angle)) {
      survey.stage = RET;
      nav_init_stage();
    }
    //return
  } else if (survey.stage == RET) {
    nav_points(survey.ret_start, survey.ret_end);
    if (nav_approaching_xy(survey.ret_end.x, survey.ret_end.y, survey.ret_start.x, survey.ret_start.y, 0)) {
      survey.stage = TURN2;
      nav_init_stage();
    }
    //turn from return to stage
  } else if (survey.stage == TURN2) {
    nav_circle_XY(survey.seg_center2.x, survey.seg_center2.y, -(2 * survey.psa_min_rad + survey.psa_sweep_width) * 0.5);
    if (NavCourseCloseTo(survey.segment_angle)) {
      survey.stage = SEG;
      nav_init_stage();
#ifdef DIGITAL_CAM
      dc_survey(survey.psa_shot_dist, survey.seg_start.x - survey.dir_vec.x * survey.psa_shot_dist * 0.5,
                survey.seg_start.y - survey.dir_vec.y * survey.psa_shot_dist * 0.5);
#endif
    }
  }

  return true;
}
