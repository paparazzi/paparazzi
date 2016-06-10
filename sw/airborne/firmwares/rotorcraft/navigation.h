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

#include "subsystems/navigation/waypoints.h"
#include "subsystems/navigation/common_flight_plan.h"

#define NAV_FREQ 16

extern struct EnuCoor_i navigation_target;
extern struct EnuCoor_i navigation_carrot;

extern void nav_init(void);
extern void nav_run(void);

extern uint8_t last_wp __attribute__((unused));

extern uint8_t horizontal_mode;
extern struct EnuCoor_i nav_segment_start, nav_segment_end;
extern struct EnuCoor_i nav_circle_center;
extern int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;
#define HORIZONTAL_MODE_WAYPOINT  0
#define HORIZONTAL_MODE_ROUTE     1
#define HORIZONTAL_MODE_CIRCLE    2
#define HORIZONTAL_MODE_ATTITUDE  3
#define HORIZONTAL_MODE_MANUAL    4
extern int32_t nav_roll, nav_pitch;     ///< with #INT32_ANGLE_FRAC
extern int32_t nav_heading; ///< with #INT32_ANGLE_FRAC
extern float nav_radius;
extern float nav_climb_vspeed, nav_descend_vspeed;

extern int32_t nav_leg_progress;
extern uint32_t nav_leg_length;

extern bool nav_survey_active;

extern uint8_t vertical_mode;
extern uint32_t nav_throttle;  ///< direct throttle from 0:MAX_PPRZ, used in VERTICAL_MODE_MANUAL
extern int32_t nav_climb, nav_altitude, nav_flight_altitude;
extern float flight_altitude;
#define VERTICAL_MODE_MANUAL      0
#define VERTICAL_MODE_CLIMB       1
#define VERTICAL_MODE_ALT         2

extern float dist2_to_home;      ///< squared distance to home waypoint
extern bool too_far_from_home;
extern float failsafe_mode_dist2; ///< maximum squared distance to home wp before going to failsafe mode

extern float dist2_to_wp;       ///< squared distance to next waypoint

extern float get_dist2_to_waypoint(uint8_t wp_id);
extern float get_dist2_to_point(struct EnuCoor_i *p);
extern void compute_dist2_to_home(void);
extern void nav_home(void);
extern void nav_set_manual(float roll, float pitch, float yaw);

unit_t nav_reset_reference(void) __attribute__((unused));
unit_t nav_reset_alt(void) __attribute__((unused));
void nav_periodic_task(void);
bool nav_detect_ground(void);
bool nav_is_in_flight(void);

extern bool exception_flag[10];
extern void set_exception_flag(uint8_t flag_num);

extern bool nav_set_heading_rad(float rad);
extern bool nav_set_heading_deg(float deg);
extern bool nav_set_heading_towards(float x, float y);
extern bool nav_set_heading_towards_waypoint(uint8_t wp);
extern bool nav_set_heading_current(void);

/** default approaching_time for a wp */
#ifndef CARROT
#define CARROT 0
#endif

#define NavKillThrottle() ({ if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(FALSE); } false; })
#define NavResurrect() ({ if (autopilot_mode == AP_MODE_NAV) { autopilot_set_motors_on(TRUE); } false; })


#define NavSetGroundReferenceHere() ({ nav_reset_reference(); false; })
#define NavSetAltitudeReferenceHere() ({ nav_reset_alt(); false; })

#define NavSetWaypointHere(_wp) ({ waypoint_set_here_2d(_wp); false; })
#define NavCopyWaypoint(_wp1, _wp2) ({ waypoint_copy(_wp1, _wp2); false; })
#define NavCopyWaypointPositionOnly(_wp1, _wp2) ({ waypoint_position_copy(_wp1, _wp2); false; })

/** Normalize a degree angle between 0 and 359 */
#define NormCourse(x) { \
    while (x < 0) x += 360; \
    while (x >= 360) x -= 360; \
  }

/*********** Navigation to  waypoint *************************************/
#define NavGotoWaypoint(_wp) { \
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT; \
    VECT3_COPY(navigation_target, waypoints[_wp].enu_i); \
    dist2_to_wp = get_dist2_to_waypoint(_wp); \
  }

/*********** Navigation on a circle **************************************/
extern void nav_circle(struct EnuCoor_i *wp_center, int32_t radius);
#define NavCircleWaypoint(_center, _radius) { \
    horizontal_mode = HORIZONTAL_MODE_CIRCLE; \
    nav_circle(&waypoints[_center].enu_i, POS_BFP_OF_REAL(_radius)); \
  }

#define NavCircleCount() ((float)abs(nav_circle_radians) / INT32_ANGLE_2_PI)
#define NavCircleQdr() ({ int32_t qdr = INT32_DEG_OF_RAD(INT32_ANGLE_PI_2 - nav_circle_qdr) >> INT32_ANGLE_FRAC; NormCourse(qdr); qdr; })

#define CloseDegAngles(_c1, _c2) ({ int32_t _diff = _c1 - _c2; NormCourse(_diff); 350 < _diff || _diff < 10; })
/** True if x (in degrees) is close to the current QDR (less than 10 degrees)*/
#define NavQdrCloseTo(x) CloseDegAngles(((x) >> INT32_ANGLE_FRAC), NavCircleQdr())
#define NavCourseCloseTo(x) {}

enum oval_status { OR12, OC2, OR21, OC1 };

extern void nav_oval_init(void);
extern void nav_oval(uint8_t, uint8_t, float);
extern uint8_t nav_oval_count;
#define Oval(a, b, c) nav_oval((b), (a), (c))

/*********** Navigation along a line *************************************/
extern void nav_route(struct EnuCoor_i *wp_start, struct EnuCoor_i *wp_end);
#define NavSegment(_start, _end) { \
    horizontal_mode = HORIZONTAL_MODE_ROUTE; \
    nav_route(&waypoints[_start].enu_i, &waypoints[_end].enu_i); \
  }

/** Nav glide routine */
#define NavGlide(_last_wp, _wp) { \
    int32_t start_alt = waypoints[_last_wp].enu_i.z; \
    int32_t diff_alt = waypoints[_wp].enu_i.z - start_alt; \
    int32_t alt = start_alt + ((diff_alt * nav_leg_progress) / nav_leg_length); \
    NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(alt),0); \
  }

/** Proximity tests on approaching a wp */
bool nav_approaching_from(struct EnuCoor_i *wp, struct EnuCoor_i *from, int16_t approaching_time);
#define NavApproaching(wp, time) nav_approaching_from(&waypoints[wp].enu_i, NULL, time)
#define NavApproachingFrom(wp, from, time) nav_approaching_from(&waypoints[wp].enu_i, &waypoints[from].enu_i, time)

/** Check the time spent in a radius of 'ARRIVED_AT_WAYPOINT' around a wp  */
bool nav_check_wp_time(struct EnuCoor_i *wp, uint16_t stay_time);
#define NavCheckWaypointTime(wp, time) nav_check_wp_time(&waypoints[wp].enu_i, time)

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

/** Set the heading of the rotorcraft, nothing else */
#define NavHeading nav_set_heading_rad

#define NavAttitude(_roll) { \
    horizontal_mode = HORIZONTAL_MODE_ATTITUDE; \
    nav_roll = ANGLE_BFP_OF_REAL(_roll); \
  }

#define NavSetManual nav_set_manual

#define NavStartDetectGround() ({ autopilot_detect_ground_once = true; false; })
#define NavDetectGround() nav_detect_ground()

#define nav_IncreaseShift(x) {}

#define nav_SetNavRadius(x) {}


#define navigation_SetFlightAltitude(x) { \
    flight_altitude = x; \
    nav_flight_altitude = POS_BFP_OF_REAL(flight_altitude - state.ned_origin_f.hmsl); \
  }

/// Get current x (east) position in local coordinates
#define GetPosX() (stateGetPositionEnu_f()->x)
/// Get current y (north) position in local coordinates
#define GetPosY() (stateGetPositionEnu_f()->y)
/// Get current altitude above MSL
#define GetPosAlt() (stateGetPositionEnu_f()->z+state.ned_origin_f.hmsl)
/**
 * Get current altitude reference for local coordinates.
 * This is the ground_alt from the flight plan at first,
 * but might be updated later through a call to NavSetGroundReferenceHere() or
 * NavSetAltitudeReferenceHere(), e.g. in the GeoInit flight plan block.
 */
#define GetAltRef() (state.ned_origin_f.hmsl)


extern void navigation_update_wp_from_speed(uint8_t wp, struct Int16Vect3 speed_sp, int16_t heading_rate_sp);

#endif /* NAVIGATION_H */
