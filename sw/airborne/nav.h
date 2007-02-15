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

#include "std.h"
#include "paparazzi.h"
#include "airframe.h"
#include "fw_v_ctl.h"

#define G 9.806
#define Square(_x) ((_x)*(_x))
#define DistanceSquare(p1_x, p1_y, p2_x, p2_y) (Square(p1_x-p2_x)+Square(p1_y-p2_y))

struct point {
  float x;
  float y;
  float a;
};

#define WaypointX(_wp) (waypoints[_wp].x)
#define WaypointY(_wp) (waypoints[_wp].y)
#define WaypointAlt(_wp) (waypoints[_wp].a)

extern float cur_pos_x;
extern float cur_pos_y;
extern float last_x, last_y;
extern uint8_t nav_stage, nav_block;
extern float dist2_to_wp, dist2_to_home;

extern int32_t nav_utm_east0; 
extern int32_t nav_utm_north0;
extern uint8_t nav_utm_zone0;

extern const uint8_t nb_waypoint;
extern struct point waypoints[]; /** size == nb_waypoint + 1 */

extern float desired_x, desired_y, altitude_shift, nav_altitude, flight_altitude, nav_glide_pitch_trim;

extern pprz_t nav_throttle_setpoint;
extern float nav_pitch, rc_pitch;
extern bool_t too_far_from_home;

/** in second */
extern uint16_t stage_time, block_time;

/** in second */
extern float stage_time_ds;

extern float carrot_x, carrot_y;

extern bool_t nav_in_circle;
extern bool_t nav_in_segment;
extern int16_t nav_circle_x, nav_circle_y, nav_circle_radius;
extern int16_t nav_segment_x_1, nav_segment_y_1, nav_segment_x_2, nav_segment_y_2;

extern uint8_t horizontal_mode;

#define HORIZONTAL_MODE_WAYPOINT 0
#define HORIZONTAL_MODE_ROUTE 1
#define HORIZONTAL_MODE_CIRCLE 2

#define MoveWaypoint(_id, _ux, _uy, _a) { \
  if (_id < nb_waypoint) { \
    waypoints[_id].x = _ux - nav_utm_east0; \
    waypoints[_id].y = _uy - nav_utm_north0; \
    waypoints[_id].a = _a; \
  } \
}

extern void nav_eight_init( void );
extern void nav_eight(uint8_t, uint8_t, float);
#define Eight(a, b, c) nav_eight((a), (b), (c))

extern void nav_oval_init( void );
extern void nav_oval(uint8_t, uint8_t, float);
#define Oval(a, b, c) nav_oval((b), (a), (c))

extern float ground_alt;

extern float survey_west, survey_east, survey_north, survey_south;


void nav_update(void);
void nav_home(void);
void nav_init(void);
void nav_without_gps(void);

extern void nav_goto_block(uint8_t block_id);

#define NavSetWaypointHere(_wp) ({ \
  waypoints[_wp].x = estimator_x; \
  waypoints[_wp].y = estimator_y; \
  FALSE; \
})

extern float nav_circle_trigo_qdr; /** Angle from center to mobile */
extern void nav_circle_XY(float x, float y, float radius);

#define NavCircleWaypoint(wp, radius) \
  nav_circle_XY(waypoints[wp].x, waypoints[wp].y, radius)

#define NormCourse(x) { \
  while (x < 0) x += 360; \
  while (x >= 360) x -= 360; \
}

#define NavCircleCount() (fabs(nav_circle_radians) / (2*M_PI))
#define NavCircleQdr() ({ float qdr = DegOfRad(M_PI_2 - nav_circle_trigo_qdr); NormCourse(qdr); qdr; })
#define NavQdrCloseTo(x) ({ float _course = x; NormCourse(_course); float circle_qdr = NavCircleQdr(); (Min(_course, 350) < circle_qdr && circle_qdr < _course+10); })

extern void nav_init_stage( void );
#define InitStage() { nav_init_stage(); return; }


/*********** Navigation along a line *************************************/
extern void nav_route_xy(float last_wp_x, float last_wp_y, float wp_x, float wp_y);
#define NavSegment(_start, _end) \
  nav_route_xy(waypoints[_start].x, waypoints[_start].y, waypoints[_end].x, waypoints[_end].y)

bool_t nav_approaching_xy(float x, float y, float from_x, float from_y, float approaching_time);
#define NavApproaching(wp, time) nav_approaching_xy(waypoints[wp].x, waypoints[wp].y, last_x, last_y, time)
#define NavApproachingFrom(wp, from, time) nav_approaching_xy(waypoints[wp].x, waypoints[wp].y, waypoints[from].x, waypoints[from].y, time)

/** Set the climb control to auto-throttle with the specified pitch
    pre-command */
#define NavVerticalAutoThrottleMode(_pitch) { \
  v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE; \
  nav_pitch = _pitch; \
}

/** Set the climb control to auto-pitch with the specified throttle
    pre-command */
#define NavVerticalAutoPitchMode(_throttle) { \
  v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_PITCH; \
  nav_throttle_setpoint = _throttle; \
}

/** Set the vertical mode to altitude control with the specified altitude
 setpoint and climb pre-command. */
#define NavVerticalAltitudeMode(_alt, _pre_climb) { \
  v_ctl_mode = V_CTL_MODE_AUTO_ALT; \
  nav_altitude = _alt; \
  v_ctl_altitude_pre_climb = _pre_climb; \
}

/** Set the vertical mode to climb control with the specified climb setpoint */
#define NavVerticalClimbMode(_climb) { \
  v_ctl_mode = V_CTL_MODE_AUTO_CLIMB; \
  v_ctl_climb_setpoint = _climb; \
}

/** Set the vertical mode to fixed throttle with the specified setpoint */
#define NavVerticalThrottleMode(_throttle) { \
  v_ctl_mode = V_CTL_MODE_AUTO_THROTTLE; \
  nav_throttle_setpoint = _throttle; \
}



#endif /* NAV_H */
