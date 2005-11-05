/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 *  \brief @@@@@ A FIXER @@@@@
 *
 */

#ifndef NAV_H
#define NAV_H

#include <std.h>

struct point {
  float x;
  float y;
  float a;
};
extern float cur_pos_x;
extern float cur_pos_y;
extern uint8_t nav_stage, nav_block;
extern float dist2_to_wp, dist2_to_home;

extern const int32_t nav_utm_east0; 
extern const int32_t nav_utm_north0;
extern const int8_t nav_utm_zone0;

extern const uint8_t nb_waypoint;
extern struct point waypoints[];
extern float desired_altitude, desired_x, desired_y;

extern uint16_t nav_desired_gaz;
extern float nav_pitch, rc_pitch;
extern bool_t too_far_from_home;
/** in seconde */
extern uint16_t stage_time, block_time;
/** in seconde */
extern float stage_time_ds;
/** in number of circle */
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


void nav_update(void);
void nav_home(void);
void nav_init(void);
void nav_without_gps(void);

#define MoveWaypoint(_id, _ux, _uy, _a) { \
  if (_id < nb_waypoint) { \
    waypoints[_id].x = _ux - nav_utm_east0; \
    waypoints[_id].y = _uy - nav_utm_north0; \
    waypoints[_id].a = _a; \
    /*** printf("%d:x=%.0f y=%.0f a=%.0f\n",_id, waypoints[_id].x, waypoints[_id].y, _a); ***/  \
  } \
}


#endif /* NAV_H */
