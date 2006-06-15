/*
 * Paparazzi mcu0 $Id$
 *  
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
 *
 */

/** \file nav.h
 *  \brief Navigation library
 * 
 * This collection of macros and functions is used by the C code generated
 * from the XML flight plan
 */

#ifndef NAV_H
#define NAV_H

#include <std.h>

#define Square(_x) ((_x)*(_x))

struct point {
  float x;
  float y;
  float a;
};

extern float cur_pos_x;
extern float cur_pos_y;
extern uint8_t nav_stage, nav_block;
extern float dist2_to_wp, dist2_to_home;

extern int32_t nav_utm_east0; 
extern int32_t nav_utm_north0;
extern uint8_t nav_utm_zone0;

extern const uint8_t nb_waypoint;
extern struct point waypoints[]; /** size == nb_waypoint + 1 */
extern bool_t moved_waypoints[]; /** size == nb_waypoint + 1 */

extern float desired_altitude, desired_x, desired_y;

extern uint16_t nav_desired_gaz;
extern float nav_pitch, rc_pitch;
extern bool_t too_far_from_home;

/** in second */
extern uint16_t stage_time, block_time;

/** in second */
extern float stage_time_ds;

/** One full circle == 1. */
extern float circle_count;

extern float nav_desired_roll;
extern float carrot_x, carrot_y;

extern bool_t in_circle;
extern bool_t in_segment;
extern int16_t circle_x, circle_y, circle_radius;
extern int16_t segment_x_1, segment_y_1, segment_x_2, segment_y_2;

extern uint8_t horizontal_mode;

#define HORIZONTAL_MODE_WAYPOINT 0
#define HORIZONTAL_MODE_ROUTE 1
#define HORIZONTAL_MODE_CIRCLE 2

#define MoveWaypoint(_id, _ux, _uy, _a) { \
  if (_id < nb_waypoint) { \
    waypoints[_id].x = _ux - nav_utm_east0; \
    waypoints[_id].y = _uy - nav_utm_north0; \
    waypoints[_id].a = _a; \
    moved_waypoints[_id] = TRUE; \
  } \
}

extern float course_pgain;
extern float desired_course;
void course_pid_run( void );

extern float climb_pgain;
extern float climb_igain;
extern float climb_sum_err;
extern float desired_climb, pre_climb;

extern float pitch_of_vz_pgain;
extern float pitch_of_vz;
extern float aileron_of_gaz;
extern float climb_level_gaz;
extern float nav_prebank, nav_speed_depend;
extern float ground_alt;

extern float survey_west, survey_east, survey_north, survey_south;

void climb_pid_run(void);
void altitude_pid_run(void);

void nav_update(void);
void nav_home(void);
void nav_init(void);
void nav_without_gps(void);

extern void nav_goto_block(uint8_t block_id);

#endif /* NAV_H */
