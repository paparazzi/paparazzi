/*
 * Copyright (C) 2017  The Paparazzi Team
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
 * @file modules/guidance/gvf/nav_survey_polygon_gvf.c
 *
 * Advanced polygon survey for fixedwings from Uni Stuttgart
 * adapted for being used with the Guidance Vector Field.
 *
 */

#include "nav_survey_polygon_gvf.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "modules/guidance/gvf/gvf.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

float gvf_nav_survey_sweep = 100.f; // dummy non-zero value, will be set at setup

struct gvf_SurveyPolyAdv gvf_survey;

static void gvf_nav_points(struct FloatVect2 start, struct FloatVect2 end)
{
  gvf_segment_XY1_XY2(start.x, start.y, end.x, end.y);
}

/**
 * intercept two lines and give back the point of intersection
 * @return         FALSE if no intersection can be found or intersection does not lie between points a and b
 * else TRUE
 * @param p               returns intersection
 * @param x, y            first line is defined by point x and y (goes through this points)
 * @param a1, a2, b1, b2  second line by coordinates a1/a2, b1/b2
 */
static bool gvf_intercept_two_lines(struct FloatVect2 *p, struct FloatVect2 x, struct FloatVect2 y, float a1, float a2,
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
static bool gvf_get_two_intersects(struct FloatVect2 *x, struct FloatVect2 *y, struct FloatVect2 a, struct FloatVect2 b)
{
  int i, count = 0;
  struct FloatVect2 tmp;

  for (i = 0; i < gvf_survey.poly_count - 1; i++)
    if (gvf_intercept_two_lines(&tmp, a, b, waypoints[gvf_survey.poly_first + i].x, waypoints[gvf_survey.poly_first + i].y,
                                waypoints[gvf_survey.poly_first + i + 1].x, waypoints[gvf_survey.poly_first + i + 1].y)) {
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
      && gvf_intercept_two_lines(&tmp, a, b, waypoints[gvf_survey.poly_first + gvf_survey.poly_count - 1].x,
                                 waypoints[gvf_survey.poly_first + gvf_survey.poly_count - 1].y, waypoints[gvf_survey.poly_first].x,
                                 waypoints[gvf_survey.poly_first].y)) {
    *y = tmp;
    count++;
  }

  if (count != 2) {
    return false;
  }

  //change points
  if (fabs(gvf_survey.dir_vec.x) > fabs(gvf_survey.dir_vec.y)) {
    if ((y->x - x->x) / gvf_survey.dir_vec.x < 0.0) {
      tmp = *x;
      *x = *y;
      *y = tmp;
    }
  } else if ((y->y - x->y) / gvf_survey.dir_vec.y < 0.0) {
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
void gvf_nav_survey_polygon_setup(uint8_t first_wp, uint8_t size, float angle, float sweep_width, float shot_dist,
                                  float min_rad, float altitude)
{
  int i;
  struct FloatVect2 small, sweep;
  float divider, angle_rad = angle / 180.0 * M_PI;

  if (angle < 0.0) { angle += 360.0; }
  if (angle >= 360.0) { angle -= 360.0; }

  gvf_survey.poly_first = first_wp;
  gvf_survey.poly_count = size;

  gvf_nav_survey_sweep = sweep_width;
  gvf_survey.psa_sweep_width = sweep_width;
  gvf_survey.psa_min_rad = min_rad;
  gvf_survey.psa_shot_dist = shot_dist;
  gvf_survey.psa_altitude = altitude;

  gvf_survey.segment_angle = angle;
  gvf_survey.return_angle = angle + 180;
  if (gvf_survey.return_angle > 359) { gvf_survey.return_angle -= 360; }

  if (angle <= 45.0 || angle >= 315.0) {
    //north
    gvf_survey.dir_vec.y = 1.0;
    gvf_survey.dir_vec.x = 1.0 * tanf(angle_rad);
    sweep.x = 1.0;
    sweep.y = - gvf_survey.dir_vec.x / gvf_survey.dir_vec.y;
  } else if (angle <= 135.0) {
    //east
    gvf_survey.dir_vec.x = 1.0;
    gvf_survey.dir_vec.y = 1.0 / tanf(angle_rad);
    sweep.y = - 1.0;
    sweep.x = gvf_survey.dir_vec.y / gvf_survey.dir_vec.x;
  } else if (angle <= 225.0) {
    //south
    gvf_survey.dir_vec.y = -1.0;
    gvf_survey.dir_vec.x = -1.0 * tanf(angle_rad);
    sweep.x = -1.0;
    sweep.y = gvf_survey.dir_vec.x / gvf_survey.dir_vec.y;
  } else {
    //west
    gvf_survey.dir_vec.x = -1.0;
    gvf_survey.dir_vec.y = -1.0 / tanf(angle_rad);
    sweep.y = 1.0;
    sweep.x = - gvf_survey.dir_vec.y / gvf_survey.dir_vec.x;
  }

  //normalize
  FLOAT_VECT2_NORMALIZE(sweep);

  VECT2_SMUL(gvf_survey.rad_vec, sweep, gvf_survey.psa_min_rad);
  VECT2_SMUL(gvf_survey.sweep_vec, sweep, gvf_survey.psa_sweep_width);

  //begin at leftmost position (relative to gvf_survey.dir_vec)
  VECT2_COPY(small, waypoints[gvf_survey.poly_first]);

  divider = (gvf_survey.sweep_vec.y * gvf_survey.dir_vec.x) - (gvf_survey.sweep_vec.x * gvf_survey.dir_vec.y);

  //calculate the leftmost point if one sees the dir vec as going "up" and the sweep vec as going right
  if (divider < 0.0) {
    for (i = 1; i < gvf_survey.poly_count; i++) {
      if ((gvf_survey.dir_vec.x * (waypoints[gvf_survey.poly_first + i].y - small.y)) + (gvf_survey.dir_vec.y *
          (small.x - waypoints[gvf_survey.poly_first + i].x)) > 0.0) {
        VECT2_COPY(small, waypoints[gvf_survey.poly_first + i]);
      }
    }
  } else {
    for (i = 1; i < gvf_survey.poly_count; i++) {
      if ((gvf_survey.dir_vec.x * (waypoints[gvf_survey.poly_first + i].y - small.y)) + (gvf_survey.dir_vec.y *
          (small.x - waypoints[gvf_survey.poly_first + i].x)) > 0.0) {
        VECT2_COPY(small, waypoints[gvf_survey.poly_first + i]);
      }
    }
  }

  //calculate the line the defines the first flyover
  gvf_survey.seg_start.x = small.x + 0.5 * gvf_survey.sweep_vec.x;
  gvf_survey.seg_start.y = small.y + 0.5 * gvf_survey.sweep_vec.y;
  VECT2_SUM(gvf_survey.seg_end, gvf_survey.seg_start, gvf_survey.dir_vec);

  if (!gvf_get_two_intersects(&gvf_survey.seg_start, &gvf_survey.seg_end, gvf_survey.seg_start, gvf_survey.seg_end)) {
    gvf_survey.stage = gERR;
    return;
  }

  //center of the entry circle
  VECT2_DIFF(gvf_survey.entry_center, gvf_survey.seg_start, gvf_survey.rad_vec);

  //fast climbing to desired altitude
  NavVerticalAutoThrottleMode(0.0);
  NavVerticalAltitudeMode(gvf_survey.psa_altitude, 0.0);

  gvf_survey.stage = gENTRY;
}

/**
 * main navigation routine. This is called periodically evaluates the current
 * Position and stage and navigates accordingly.
 * @returns True until the survey is finished
 */
void gvf_nav_direction_circle(float rad)
{
  if (rad > 0) {
    gvf_set_direction(-1);
  } else {
    gvf_set_direction(1);
  }
}

bool gvf_nav_survey_polygon_run(void)
{
  #ifdef NAV_SURVEY_POLY_GVF_DYNAMIC
  sweep_width = (nav_survey_shift > 0 ? gvf_nav_survey_sweep : -gvf_nav_survey_sweep);
  #endif

  NavVerticalAutoThrottleMode(0.0);
  NavVerticalAltitudeMode(gvf_survey.psa_altitude, 0.0);

  //entry circle around entry-center until the desired altitude is reached
  if (gvf_survey.stage == gENTRY) {
    gvf_nav_direction_circle(gvf_survey.psa_min_rad);
    gvf_ellipse_XY(gvf_survey.entry_center.x, gvf_survey.entry_center.y, gvf_survey.psa_min_rad, gvf_survey.psa_min_rad, 0);
    if (NavCourseCloseTo(gvf_survey.segment_angle)
        && nav_approaching_xy(gvf_survey.seg_start.x, gvf_survey.seg_start.y, last_x, last_y, CARROT)
        && fabs(stateGetPositionUtm_f()->alt - gvf_survey.psa_altitude) <= 20) {
      gvf_survey.stage = gSEG;
      nav_init_stage();
#ifdef DIGITAL_CAM
      dc_survey(gvf_survey.psa_shot_dist, gvf_survey.seg_start.x - gvf_survey.dir_vec.x * gvf_survey.psa_shot_dist * 0.5,
                gvf_survey.seg_start.y - gvf_survey.dir_vec.y * gvf_survey.psa_shot_dist * 0.5);
#endif
    }
  }
  //fly the segment until seg_end is reached
  if (gvf_survey.stage == gSEG) {
    gvf_nav_points(gvf_survey.seg_start, gvf_survey.seg_end);
    //calculate all needed points for the next flyover
    if (nav_approaching_xy(gvf_survey.seg_end.x, gvf_survey.seg_end.y, gvf_survey.seg_start.x, gvf_survey.seg_start.y, 0)) {
#ifdef DIGITAL_CAM
      dc_stop();
#endif
      VECT2_DIFF(gvf_survey.seg_center1, gvf_survey.seg_end, gvf_survey.rad_vec);
      gvf_survey.ret_start.x = gvf_survey.seg_end.x - 2 * gvf_survey.rad_vec.x;
      gvf_survey.ret_start.y = gvf_survey.seg_end.y - 2 * gvf_survey.rad_vec.y;

      //if we get no intersection the survey is finished
      static struct FloatVect2 sum_start_sweep;
      static struct FloatVect2 sum_end_sweep;
      VECT2_SUM(sum_start_sweep, gvf_survey.seg_start, gvf_survey.sweep_vec);
      VECT2_SUM(sum_end_sweep, gvf_survey.seg_end, gvf_survey.sweep_vec);
      if (!gvf_get_two_intersects(&gvf_survey.seg_start, &gvf_survey.seg_end, sum_start_sweep, sum_end_sweep)) {
        return false;
      }

      gvf_survey.ret_end.x = gvf_survey.seg_start.x - gvf_survey.sweep_vec.x - 2 * gvf_survey.rad_vec.x;
      gvf_survey.ret_end.y = gvf_survey.seg_start.y - gvf_survey.sweep_vec.y - 2 * gvf_survey.rad_vec.y;

      gvf_survey.seg_center2.x = gvf_survey.seg_start.x - 0.5 * (2.0 * gvf_survey.rad_vec.x + gvf_survey.sweep_vec.x);
      gvf_survey.seg_center2.y = gvf_survey.seg_start.y - 0.5 * (2.0 * gvf_survey.rad_vec.y + gvf_survey.sweep_vec.y);

      gvf_survey.stage = gTURN1;
      nav_init_stage();
    }
  }
  //turn from stage to return
  else if (gvf_survey.stage == gTURN1) {
    gvf_nav_direction_circle(gvf_survey.psa_min_rad);
    gvf_ellipse_XY(gvf_survey.seg_center1.x, gvf_survey.seg_center1.y, gvf_survey.psa_min_rad, gvf_survey.psa_min_rad, 0);
    if (NavCourseCloseTo(gvf_survey.return_angle)) {
      gvf_survey.stage = gRET;
      nav_init_stage();
    }
    //return
  } else if (gvf_survey.stage == gRET) {
    gvf_nav_points(gvf_survey.ret_start, gvf_survey.ret_end);
    if (nav_approaching_xy(gvf_survey.ret_end.x, gvf_survey.ret_end.y, gvf_survey.ret_start.x, gvf_survey.ret_start.y, 0)) {
      gvf_survey.stage = gTURN2;
      nav_init_stage();
    }
    //turn from return to stage
  } else if (gvf_survey.stage == gTURN2) {
    float rad_sur = (2 * gvf_survey.psa_min_rad + gvf_survey.psa_sweep_width) * 0.5;
    gvf_nav_direction_circle(rad_sur);
    gvf_ellipse_XY(gvf_survey.seg_center2.x, gvf_survey.seg_center2.y, rad_sur, rad_sur, 0);
    if (NavCourseCloseTo(gvf_survey.segment_angle)) {
      gvf_survey.stage = gSEG;
      nav_init_stage();
#ifdef DIGITAL_CAM
      dc_survey(gvf_survey.psa_shot_dist, gvf_survey.seg_start.x - gvf_survey.dir_vec.x * gvf_survey.psa_shot_dist * 0.5,
                gvf_survey.seg_start.y - gvf_survey.dir_vec.y * gvf_survey.psa_shot_dist * 0.5);
#endif
    }
  }

  return true;
}
