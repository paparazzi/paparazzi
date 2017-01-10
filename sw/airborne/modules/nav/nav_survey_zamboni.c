/*
 * Copyright (C) 2013 Jorn Anke, Felix Ruess
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
 * @file modules/nav/nav_survey_zamboni.c
 *
 * Zamboni pattern survey for fixedwings.
 */

#include "modules/nav/nav_survey_zamboni.h"

#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

struct ZamboniSurvey zs;

/**
 * initializes the variables needed for the survey to start.
 *
 * @param center_wp     the waypoint defining the center of the survey-rectangle
 * @param dir_wp        the waypoint defining the orientation of the survey-rectangle
 * @param sweep_length  the length of the survey-rectangle
 * @param sweep_spacing distance between the sweeps
 * @param sweep_lines   number of sweep_lines to fly
 * @param altitude      the altitude that must be reached before the flyover starts
 */
void nav_survey_zamboni_setup(uint8_t center_wp, uint8_t dir_wp, float sweep_length, float sweep_spacing,
                              int sweep_lines, float altitude)
{
  zs.current_laps = 0;
  zs.pre_leave_angle = 2;

  // copy variables from the flight plan
  VECT2_COPY(zs.wp_center, waypoints[center_wp]);
  VECT2_COPY(zs.wp_dir, waypoints[dir_wp]);
  zs.altitude = altitude;

  // if turning right leave circle before angle is reached, if turning left - leave after
  if (sweep_spacing > 0) {
    zs.pre_leave_angle = 0;
  }

  struct FloatVect2 flight_vec;
  VECT2_DIFF(flight_vec, zs.wp_dir, zs.wp_center);
  FLOAT_VECT2_NORMALIZE(flight_vec);

  // calculate the flight_angle
  zs.flight_angle = DegOfRad(atan2(flight_vec.x, flight_vec.y));
  zs.return_angle = zs.flight_angle + 180;
  if (zs.return_angle > 359) {
    zs.return_angle -= 360;
  }

  // calculate the vector from one flightline perpendicular to the next flightline left,
  // seen in the flightdirection. (Delta x and delta y betwen two adjecent flightlines)
  // (used later to move the flight pattern one flightline up for each round)
  zs.sweep_width.x = -flight_vec.y * sweep_spacing;
  zs.sweep_width.y = +flight_vec.x * sweep_spacing;

  // calculate number of laps to fly and turning radius for each end
  zs.total_laps = (sweep_lines + 1) / 2;
  zs.turnradius1 = (zs.total_laps - 1) * sweep_spacing * 0.5;
  zs.turnradius2 = zs.total_laps * sweep_spacing * 0.5;

  struct FloatVect2 flight_line;
  VECT2_SMUL(flight_line, flight_vec, sweep_length * 0.5);

  //CALCULATE THE NAVIGATION POINTS
  //start and end of flight-line in flight-direction
  VECT2_DIFF(zs.seg_start, zs.wp_center, flight_line);
  VECT2_SUM(zs.seg_end, zs.wp_center, flight_line);

  struct FloatVect2 sweep_span;
  VECT2_SMUL(sweep_span, zs.sweep_width, zs.total_laps - 1);
  //start and end of flight-line in return-direction
  VECT2_DIFF(zs.ret_start, zs.seg_end, sweep_span);
  VECT2_DIFF(zs.ret_end, zs.seg_start, sweep_span);

  //turn-centers at both ends
  zs.turn_center1.x = (zs.seg_end.x + zs.ret_start.x) / 2.0;
  zs.turn_center1.y = (zs.seg_end.y + zs.ret_start.y) / 2.0;
  zs.turn_center2.x = (zs.seg_start.x + zs.ret_end.x + zs.sweep_width.x) / 2.0;
  zs.turn_center2.y = (zs.seg_start.y + zs.ret_end.y + zs.sweep_width.y) / 2.0;

  //fast climbing to desired altitude
  NavVerticalAutoThrottleMode(100.0);
  NavVerticalAltitudeMode(zs.altitude, 0.0);

  zs.stage = Z_ENTRY;
}

/**
 * main navigation routine.
 * This is called periodically evaluates the current
 * Position and stage and navigates accordingly.
 *
 * @returns TRUE until the survey is finished
 */
bool nav_survey_zamboni_run(void)
{
  // retain altitude
  NavVerticalAutoThrottleMode(0.0);
  NavVerticalAltitudeMode(zs.altitude, 0.0);

  //go from center of field to end of field - (before starting the syrvey)
  if (zs.stage == Z_ENTRY) {
    nav_route_xy(zs.wp_center.x, zs.wp_center.y, zs.seg_end.x, zs.seg_end.y);
    if (nav_approaching_xy(zs.seg_end.x, zs.seg_end.y, zs.wp_center.x, zs.wp_center.y, CARROT)) {
      zs.stage = Z_TURN1;
      NavVerticalAutoThrottleMode(0.0);
      nav_init_stage();
    }
  }

  //Turn from stage to return
  else if (zs.stage == Z_TURN1) {
    nav_circle_XY(zs.turn_center1.x, zs.turn_center1.y, zs.turnradius1);
    if (NavCourseCloseTo(zs.return_angle + zs.pre_leave_angle)) {
      // && nav_approaching_xy(zs.seg_end.x, zs.seg_end.y, zs.seg_start.x, zs.seg_start.y, CARROT
      //calculate SEG-points for the next flyover
      VECT2_ADD(zs.seg_start, zs.sweep_width);
      VECT2_ADD(zs.seg_end, zs.sweep_width);

      zs.stage = Z_RET;
      nav_init_stage();
#ifdef DIGITAL_CAM
      LINE_START_FUNCTION;
#endif
    }
  }

  //fly the segment until seg_end is reached
  else if (zs.stage == Z_RET) {
    nav_route_xy(zs.ret_start.x, zs.ret_start.y, zs.ret_end.x, zs.ret_end.y);
    if (nav_approaching_xy(zs.ret_end.x, zs.ret_end.y, zs.ret_start.x, zs.ret_start.y, 0)) {
      zs.current_laps = zs.current_laps + 1;
#ifdef DIGITAL_CAM
      //dc_stop();
      LINE_STOP_FUNCTION;
#endif
      zs.stage = Z_TURN2;
    }
  }

  //turn from stage to return
  else if (zs.stage == Z_TURN2) {
    nav_circle_XY(zs.turn_center2.x, zs.turn_center2.y, zs.turnradius2);
    if (NavCourseCloseTo(zs.flight_angle + zs.pre_leave_angle)) {
      //zs.current_laps = zs.current_laps + 1;
      zs.stage = Z_SEG;
      nav_init_stage();
#ifdef DIGITAL_CAM
      LINE_START_FUNCTION;
#endif
    }

    //return
  } else if (zs.stage == Z_SEG) {
    nav_route_xy(zs.seg_start.x, zs.seg_start.y, zs.seg_end.x, zs.seg_end.y);
    if (nav_approaching_xy(zs.seg_end.x, zs.seg_end.y, zs.seg_start.x, zs.seg_start.y, 0)) {

      // calculate the rest of the points for the next fly-over
      VECT2_ADD(zs.ret_start, zs.sweep_width);
      VECT2_ADD(zs.ret_end, zs.sweep_width);
      zs.turn_center1.x = (zs.seg_end.x + zs.ret_start.x) / 2;
      zs.turn_center1.y = (zs.seg_end.y + zs.ret_start.y) / 2;
      zs.turn_center2.x = (zs.seg_start.x + zs.ret_end.x + zs.sweep_width.x) / 2;
      zs.turn_center2.y = (zs.seg_start.y + zs.ret_end.y + zs.sweep_width.y) / 2;

      zs.stage = Z_TURN1;
      nav_init_stage();
#ifdef DIGITAL_CAM
      //dc_stop();
      LINE_STOP_FUNCTION;
#endif
    }
  }
  if (zs.current_laps >= zs.total_laps) {
#ifdef DIGITAL_CAM
    LINE_STOP_FUNCTION;
#endif
    return false;
  } else {
    return true;
  }
}
