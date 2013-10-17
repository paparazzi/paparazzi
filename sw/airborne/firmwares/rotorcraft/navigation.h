/*
 * Copyright (C) 2008-2011  The Paparazzi Team
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
 * @file firmwares/rotorcraft/navigation.h
 *
 * Rotorcraft navigation functions.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

#include "subsystems/navigation/common_flight_plan.h"

#define NAV_FREQ 16

extern struct EnuCoor_i navigation_target;
extern struct EnuCoor_i navigation_carrot;

extern struct EnuCoor_i waypoints[];
extern const uint8_t nb_waypoint;

extern void nav_init(void);
extern void nav_run(void);

extern uint8_t last_wp __attribute__ ((unused));

extern int32_t ground_alt;

extern uint8_t horizontal_mode;
extern uint8_t nav_segment_start, nav_segment_end;
extern uint8_t nav_circle_centre;
extern int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;
#define HORIZONTAL_MODE_WAYPOINT  0
#define HORIZONTAL_MODE_ROUTE     1
#define HORIZONTAL_MODE_CIRCLE    2
#define HORIZONTAL_MODE_ATTITUDE  3
extern int32_t nav_roll, nav_pitch;     ///< with #INT32_ANGLE_FRAC
extern int32_t nav_heading, nav_course; ///< with #INT32_ANGLE_FRAC
extern float nav_radius;

extern int32_t nav_leg_progress;
extern int32_t nav_leg_length;

extern uint8_t vertical_mode;
extern uint32_t nav_throttle;  ///< direct throttle from 0:MAX_PPRZ, used in VERTICAL_MODE_MANUAL
extern int32_t nav_climb, nav_altitude, nav_flight_altitude;
extern float flight_altitude;
#define VERTICAL_MODE_MANUAL      0
#define VERTICAL_MODE_CLIMB       1
#define VERTICAL_MODE_ALT         2

extern float dist2_to_home;      ///< squared distance to home waypoint
extern bool_t too_far_from_home;

extern void compute_dist2_to_home(void);
extern void nav_home(void);

unit_t nav_reset_reference( void ) __attribute__ ((unused));
unit_t nav_reset_alt( void ) __attribute__ ((unused));
void nav_periodic_task(void);
void nav_move_waypoint_lla(uint8_t wp_id, struct LlaCoor_i* new_lla_pos);
void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i * new_pos);
bool_t nav_detect_ground(void);
bool_t nav_is_in_flight(void);


#define NavKillThrottle() ({ if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(FALSE); } FALSE; })
#define NavResurrect() ({ if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(TRUE); } FALSE; })


#define NavSetGroundReferenceHere() ({ nav_reset_reference(); FALSE; })
#define NavSetAltitudeReferenceHere() ({ nav_reset_alt(); FALSE; })

#define NavSetWaypointHere(_wp) ({ VECT2_COPY(waypoints[_wp], *stateGetPositionEnu_i()); FALSE; })
#define NavCopyWaypoint(_wp1, _wp2) ({ VECT2_COPY(waypoints[_wp1], waypoints[_wp2]); FALSE; })

#define WaypointX(_wp)    POS_FLOAT_OF_BFP(waypoints[_wp].x)
#define WaypointY(_wp)    POS_FLOAT_OF_BFP(waypoints[_wp].y)
#define WaypointAlt(_wp)  POS_FLOAT_OF_BFP(waypoints[_wp].z)
#define Height(_h) (_h)

/** Normalize a degree angle between 0 and 359 */
#define NormCourse(x) { \
  while (x < 0) x += 360; \
  while (x >= 360) x -= 360; \
}

/*********** Navigation to  waypoint *************************************/
#define NavGotoWaypoint(_wp) { \
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT; \
  INT32_VECT3_COPY( navigation_target, waypoints[_wp]); \
}

/*********** Navigation on a circle **************************************/
extern void nav_circle(uint8_t wp_center, int32_t radius);
#define NavCircleWaypoint(_center, _radius) { \
  horizontal_mode = HORIZONTAL_MODE_CIRCLE; \
  nav_circle(_center, POS_BFP_OF_REAL(_radius)); \
}

#define NavCircleCount() (abs(nav_circle_radians) / INT32_ANGLE_2_PI)
#define NavCircleQdr() ({ int32_t qdr = INT32_DEG_OF_RAD(INT32_ANGLE_2_PI_2 - nav_circle_qdr) >> INT32_ANGLE_FRAC; NormCourse(qdr); qdr; })

/** True if x (in degrees) is close to the current QDR (less than 10 degrees)*/
#define NavQdrCloseTo(x) {}
#define NavCourseCloseTo(x) {}

/*********** Navigation along a line *************************************/
extern void nav_route(uint8_t wp_start, uint8_t wp_end);
#define NavSegment(_start, _end) { \
  horizontal_mode = HORIZONTAL_MODE_ROUTE; \
  nav_route(_start, _end); \
}

/** Nav glide routine */
#define NavGlide(_last_wp, _wp) { \
  int32_t start_alt = waypoints[_last_wp].z; \
  int32_t diff_alt = waypoints[_wp].z - start_alt; \
  int32_t alt = start_alt + ((diff_alt * nav_leg_progress) / nav_leg_length); \
  NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(alt),0); \
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
  vertical_mode = VERTICAL_MODE_MANUAL;      \
  nav_throttle = _throttle;                  \
}

#define NavHeading(_course) {}

#define NavAttitude(_roll) { \
  horizontal_mode = HORIZONTAL_MODE_ATTITUDE; \
  nav_roll = ANGLE_BFP_OF_REAL(_roll); \
}

#define NavStartDetectGround() ({ autopilot_detect_ground_once = TRUE; FALSE; })
#define NavDetectGround() nav_detect_ground()

#define nav_IncreaseShift(x) {}

#define nav_SetNavRadius(x) {}

#define navigation_SetNavHeading(x) { \
  nav_heading = ANGLE_BFP_OF_REAL(x); \
}

#define navigation_SetFlightAltitude(x) { \
  flight_altitude = x; \
  nav_flight_altitude = POS_BFP_OF_REAL(flight_altitude) - ground_alt; \
}


#define GetPosX() (stateGetPositionEnu_f()->x)
#define GetPosY() (stateGetPositionEnu_f()->y)
#define GetPosAlt() (stateGetPositionEnu_f()->z+ground_alt)


extern void navigation_update_wp_from_speed(uint8_t wp, struct Int16Vect3 speed_sp, int16_t heading_rate_sp );

#endif /* NAVIGATION_H */
