/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ2_NAVIGATION_H
#define BOOZ2_NAVIGATION_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

extern struct EnuCoor_i booz2_navigation_target;
extern struct EnuCoor_i booz2_navigation_carrot;

extern struct EnuCoor_f waypoints_float[];
extern struct EnuCoor_i waypoints[];
extern const uint8_t nb_waypoint;

extern void booz2_nav_init(void);
extern void booz2_nav_run(void);

extern uint16_t stage_time, block_time;

extern uint8_t nav_stage, nav_block;
extern uint8_t last_block, last_stage;
extern uint8_t last_wp __attribute__ ((unused));

extern int32_t ground_alt;

extern uint8_t horizontal_mode;
extern uint8_t nav_segment_start, nav_segment_end;
#define HORIZONTAL_MODE_WAYPOINT  0
#define HORIZONTAL_MODE_ROUTE     1
#define HORIZONTAL_MODE_CIRCLE    2
#define HORIZONTAL_MODE_ATTITUDE  3
extern int32_t nav_roll, nav_pitch;
extern int32_t nav_heading, nav_course;

extern uint8_t vertical_mode;
extern uint32_t nav_throttle;
extern int32_t nav_climb, nav_altitude, nav_flight_altitude;
extern float flight_altitude;
#define VERTICAL_MODE_MANUAL      0
#define VERTICAL_MODE_CLIMB       1
#define VERTICAL_MODE_ALT         2

void nav_init_stage(void);
void nav_init_block(void);
void nav_goto_block(uint8_t block_id);
void compute_dist2_to_home(void);
unit_t nav_reset_reference( void ) __attribute__ ((unused));
unit_t nav_reset_alt( void ) __attribute__ ((unused));
void nav_periodic_task(void);
void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i * new_pos);

void nav_home(void);

#define NavKillThrottle() { kill_throttle = 1; booz2_autopilot_motors_on = 0; }
#define NavResurrect() { kill_throttle = 0; booz2_autopilot_motors_on = 1; }

#define InitStage() nav_init_stage();

#define Block(x) case x: nav_block=x;
#define NextBlock() { nav_block++; nav_init_block(); }
#define GotoBlock(b) { nav_block=b; nav_init_block(); }

#define Stage(s) case s: nav_stage=s;
#define NextStageAndBreak() { nav_stage++; InitStage(); break; }
#define NextStageAndBreakFrom(wp) { last_wp = wp; NextStageAndBreak(); }

#define Label(x) label_ ## x:
#define Goto(x) { goto label_ ## x; }
#define Return() ({ nav_block=last_block; nav_stage=last_stage; block_time=0; FALSE;})

#define And(x, y) ((x) && (y))
#define Or(x, y) ((x) || (y))
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)
#define LessThan(_x, _y) ((_x) < (_y))
#define MoreThan(_x, _y) ((_x) > (_y))

/** Time in s since the entrance in the current block */
#define NavBlockTime() (block_time)

#define NavSetGroundReferenceHere() ({ nav_reset_reference(); FALSE; })
#define NavSetAltitudeReferenceHere() ({ nav_reset_alt(); FALSE; })

#define NavSetWaypointHere(_wp) { FALSE; }

#define NavCircleWaypoint(wp, radius) {}

/** Normalize a degree angle between 0 and 359 */
#define NormCourse(x) { \
  while (x < 0) x += 360; \
  while (x >= 360) x -= 360; \
}

#define NavCircleCount() {}
#define NavCircleQdr() {}

/** True if x (in degrees) is close to the current QDR (less than 10 degrees)*/
#define NavQdrCloseTo(x) {}
#define NavCourseCloseTo(x) {}

#define WaypointX(_wp)    POS_FLOAT_OF_BFP(waypoints[_wp].x)
#define WaypointY(_wp)    POS_FLOAT_OF_BFP(waypoints[_wp].y)
#define WaypointAlt(_wp)  POS_FLOAT_OF_BFP(waypoints[_wp].z)
#define Height(_h) (_h)

/*********** Navigation to  waypoint *************************************/
#define NavGotoWaypoint(_wp) { \
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT; \
  INT32_VECT3_COPY( booz2_navigation_target, waypoints[_wp]); \
}

#define NavGlide(_last_wp, _wp) {}

/*********** Navigation along a line *************************************/
extern void nav_route(uint8_t wp_start, uint8_t wp_end);
#define NavSegment(_start, _end) { \
  horizontal_mode = HORIZONTAL_MODE_ROUTE; \
  nav_route(_start, _end); \
}


bool_t nav_approaching_from(uint8_t wp_idx, uint8_t from_idx);
#define NavApproaching(wp, time) nav_approaching_from(wp, 0)
#define NavApproachingFrom(wp, from, time) nav_approaching_from(wp, from)

/** Set the climb control to auto-throttle with the specified pitch
    pre-command */
#define NavVerticalAutoThrottleMode(_pitch) { \
  nav_pitch = ANGLE_BFP_OF_REAL(_pitch); \
}

/** Set the climb control to auto-pitch with the specified throttle
    pre-command */
#define NavVerticalAutoPitchMode(_throttle) {}

/** Set the vertical mode to altitude control with the specified altitude
 setpoint and climb pre-command. */
#define NavVerticalAltitudeMode(_alt, _pre_climb) { \
  vertical_mode = VERTICAL_MODE_ALT; \
  nav_altitude = POS_BFP_OF_REAL(_alt); \
}

/** Set the vertical mode to climb control with the specified climb setpoint */
#define NavVerticalClimbMode(_climb) { \
  vertical_mode = VERTICAL_MODE_CLIMB; \
  nav_climb = SPEED_BFP_OF_REAL(_climb); \
}

/** Set the vertical mode to fixed throttle with the specified setpoint */
#define NavVerticalThrottleMode(_throttle) { \
  vertical_mode = VERTICAL_MODE_MANUAL; \
  nav_throttle = (200 * ((uint32_t)_throttle) / 9600); \
}

#define NavHeading(_course) {}

#define NavAttitude(_roll) { \
  horizontal_mode = HORIZONTAL_MODE_ATTITUDE; \
  nav_roll = ANGLE_BFP_OF_REAL(_roll); \
}

#define NAV_GROUND_DETECT ACCEL_BFP_OF_REAL(15.)
#define NavDetectGround() ( booz_ins_enu_accel.z < -NAV_GROUND_DETECT || booz_ins_enu_accel.z > NAV_GROUND_DETECT )

#define nav_IncreaseShift(x) {}

#define nav_SetNavRadius(x) {}

#define booz2_navigation_SetNavHeading(x) { \
  nav_heading = ANGLE_BFP_OF_REAL(x); \
}

#define booz2_navigation_SetFlightAltitude(x) { \
  flight_altitude = x; \
  nav_flight_altitude = POS_BFP_OF_REAL(flight_altitude) - ground_alt; \
}

#endif /* BOOZ2_NAVIGATION_H */
