/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rover/navigation.h
 *
 * Rover navigation functions.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "std.h"
#include "math/pprz_geodetic_float.h"
#include "state.h"
#include "modules/nav/waypoints.h"
#include "modules/nav/common_flight_plan.h"
#include "autopilot.h"

/** default approaching_time for a wp */ // FIXME
#ifndef CARROT
#define CARROT 0.f
#endif

#ifndef CARROT_DIST
#define CARROT_DIST 2.f
#endif

/** default navigation frequency */
#ifndef NAVIGATION_FREQUENCY
#if PERIODIC_FREQUENCY == 512
#define NAVIGATION_FREQUENCY 16
#else // if not 512, assume a multiple of 20 (e.g. 200, 500, 1000, ...)
#define NAVIGATION_FREQUENCY 20
#endif
#endif


/** default nav_circle_radius in meters */
#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 6.0f
#endif

/** minimum horizontal distance to waypoint to mark as arrived */
#ifndef ARRIVED_AT_WAYPOINT
#define ARRIVED_AT_WAYPOINT 3.0f
#endif

/** Maximum distance from HOME waypoint before going into failsafe mode */
#ifndef FAILSAFE_MODE_DISTANCE
#define FAILSAFE_MODE_DISTANCE (1.2*MAX_DIST_FROM_HOME)
#endif

/** Nav modes */
#define NAV_MODE_WAYPOINT 0
#define NAV_MODE_ROUTE    1
#define NAV_MODE_CIRCLE   2
#define NAV_MODE_HEADING  3
#define NAV_MODE_MANUAL   4

typedef void (*nav_rover_goto)(struct EnuCoor_f *wp);
typedef void (*nav_rover_route)(struct EnuCoor_f *wp_start, struct EnuCoor_f *wp_end);
typedef bool (*nav_rover_approaching)(struct EnuCoor_f *wp_to, struct EnuCoor_f *wp_from, float approaching_time);
typedef void (*nav_rover_circle)(struct EnuCoor_f *wp_center, float radius);
typedef void (*nav_rover_oval_init)(void);
typedef void (*nav_rover_oval)(struct EnuCoor_f *wp1, struct EnuCoor_f *wp2, float radius);


/** General Navigation structure
 */
struct RoverNavigation {
  // mode
  uint8_t mode;             // nav mode

  // commands
  struct EnuCoor_f target;  ///< final target
  struct EnuCoor_f carrot;  ///< carrot position
  float heading;            ///< heading setpoint (in radians)
  float radius;             ///< radius setpoint
  float speed;              ///< speed setpoint
  float turn;               ///< turn rate setpoint
  float shift;              ///< lateral shift (in meters)

  // misc
  float dist2_to_home;        ///< squared distance to home waypoint
  bool too_far_from_home;     ///< too_far flag
  float failsafe_mode_dist2;  ///< maximum squared distance to home wp before going to failsafe mode
  bool exception_flag[10];    ///< array of flags that might be used in flight plans
  struct EnuCoor_f last_pos;  ///< last stage position

  // pointers to basic nav functions
  nav_rover_goto nav_goto;
  nav_rover_route nav_route;
  nav_rover_approaching nav_approaching;
  nav_rover_circle nav_circle;
  nav_rover_oval_init nav_oval_init;
  nav_rover_oval nav_oval;
};

extern struct RoverNavigation nav;

/** Registering functions
 */
extern void nav_register_goto_wp(nav_rover_goto nav_goto,
    nav_rover_route nav_route,
    nav_rover_approaching nav_approaching);
extern void nav_register_circle(nav_rover_circle nav_circle);
extern void nav_register_oval(nav_rover_oval_init nav_oval_init, nav_rover_oval nav_oval);
// TODO: eight, survey


/*****************************************************************
 * macros to ensure compatibility between fixedwing and rotorcraft
 *****************************************************************/

/// Get current x (east) position in local coordinates
#define GetPosX() (stateGetPositionEnu_f()->x)
/// Get current y (north) position in local coordinates
#define GetPosY() (stateGetPositionEnu_f()->y)
/// Get current altitude above MSL
#define GetPosAlt() (stateGetPositionEnu_f()->z+state.ned_origin_f.hmsl)
/// Get current height above reference
#define GetPosHeight() (stateGetPositionEnu_f()->z)
/**
 * Get current altitude reference for local coordinates.
 * This is the ground_alt from the flight plan at first,
 * but might be updated later through a call to NavSetGroundReferenceHere() or
 * NavSetAltitudeReferenceHere(), e.g. in the GeoInit flight plan block.
 */
#define GetAltRef() (state.ned_origin_f.hmsl)


/** Normalize a degree angle between 0 and 359 */
#define NormCourse(x) { \
    while (x < 0) x += 360; \
    while (x >= 360) x -= 360; \
  }
/** Normalize a rad angle between 0 and 2*PI */
#define NormCourseRad(x) { \
    while (x < 0) x += 2*M_PI; \
    while (x >= 2*M_PI) x -= 2*M_PI; \
  }

extern uint8_t last_wp __attribute__((unused));

extern void nav_init(void);
extern void nav_run(void);

extern void set_exception_flag(uint8_t flag_num);

extern float get_dist2_to_waypoint(uint8_t wp_id);
extern float get_dist2_to_point(struct EnuCoor_f *p);
extern void compute_dist2_to_home(void);
extern void nav_home(void);
extern void nav_set_manual(float speed, float turn);

extern void nav_reset_reference(void) __attribute__((unused));
extern void nav_periodic_task(void);

extern bool nav_is_in_flight(void);

/** heading utility functions */
extern void nav_set_heading_rad(float rad);
extern void nav_set_heading_deg(float deg);
extern void nav_set_heading_towards(float x, float y);
extern void nav_set_heading_towards_waypoint(uint8_t wp);
extern void nav_set_heading_towards_target(void);
extern void nav_set_heading_current(void);

extern void nav_set_failsafe(void);

/* switching motors on/off */
static inline void NavKillThrottle(void)
{
  if (autopilot_get_mode() == AP_MODE_NAV) { autopilot_set_motors_on(FALSE); }
}
static inline void NavResurrect(void)
{
  if (autopilot_get_mode() == AP_MODE_NAV) { autopilot_set_motors_on(TRUE); }
}


#define NavSetManual(_roll, _pitch, _yaw) _Pragma("GCC error \"Manual mode in flight plan for fixedwing is not available\"")
#define NavSetFailsafe nav_set_failsafe

#define NavSetGroundReferenceHere nav_reset_reference
#define NavSetAltitudeReferenceHere nav_reset_alt

#define NavSetWaypointHere waypoint_set_here_2d
#define NavCopyWaypoint waypoint_copy
#define NavCopyWaypointPositionOnly waypoint_position_copy


/** Check the time spent in a radius of 'ARRIVED_AT_WAYPOINT' around a wp  */
bool nav_check_wp_time(struct EnuCoor_f *wp, float stay_time);
#define NavCheckWaypointTime(wp, time) nav_check_wp_time(&waypoints[wp].enu_f, time)


/***********************************************************
 * macros used by flight plan to set different modes
 **********************************************************/

// command direct turn rate from FP roll [-1.; 1.]
#define NavAttitude(_roll) {  \
  nav.mode = NAV_MODE_MANUAL; \
  nav.turn = _roll;           \
  BoundAbs(nav.turn, 1.f);    \
}

// command direct motor speed from FP throttle [-1.; 1.]
#define NavVerticalThrottleMode(_speed) { \
  nav.speed = _speed;                     \
  BoundAbs(nav.speed, 1.f);               \
}

/** Set the heading of the rover, nothing else */
#define NavHeading nav_set_heading_rad



/***********************************************************
 * built in navigation routines
 **********************************************************/

/*********** Navigation to  waypoint *************************************/
static inline void NavGotoWaypoint(uint8_t wp)
{
  nav.mode = NAV_MODE_WAYPOINT;
  VECT3_COPY(nav.target, waypoints[wp].enu_f);
  //nav.dist2_to_wp = get_dist2_to_waypoint(wp); FIXME
}

/*********** Navigation along a line *************************************/
static inline void NavSegment(uint8_t wp_start, uint8_t wp_end)
{
  nav.mode = NAV_MODE_ROUTE;
  if (nav.nav_route) {
    nav.nav_route(&waypoints[wp_start].enu_f, &waypoints[wp_end].enu_f);
  }
}

static inline bool NavApproaching(uint8_t wp, float approaching_time)
{
  if (nav.nav_approaching) {
    return nav.nav_approaching(&waypoints[wp].enu_f, NULL, approaching_time);
  }
  else {
    return true;
  }
}

static inline bool NavApproachingFrom(uint8_t to, uint8_t from, float approaching_time)
{
  if (nav.nav_approaching) {
    return nav.nav_approaching(&waypoints[to].enu_f, &waypoints[from].enu_f, approaching_time);
  }
  else {
    return true;
  }
}

/*********** Navigation on a circle **************************************/
static inline void NavCircleWaypoint(uint8_t wp_center, float radius)
{
  nav.mode = NAV_MODE_CIRCLE;
  if (nav.nav_circle) {
    nav.nav_circle(&waypoints[wp_center].enu_f, radius);
  }
}

/*********** Navigation along an oval *************************************/
static inline void nav_oval_init(void)
{
  if (nav.nav_oval_init) {
    nav.nav_oval_init();
  }
}

static inline void Oval(uint8_t wp1, uint8_t wp2, float radius)
{
  if (nav.nav_oval) {
    nav.nav_oval(&waypoints[wp1].enu_f, &waypoints[wp2].enu_f, radius);
  }
}


/** Settings handlers
 */
#define navigation_IncreaseShift(x) { if (x==0) nav.shift = 0; else nav.shift += x; }

#define navigation_SetNavRadius(x) { if (x==1) nav.radius = DEFAULT_CIRCLE_RADIUS; else if (x==-1) nav.radius = -DEFAULT_CIRCLE_RADIUS; else nav.radius = x; }


/* follow another aircraft TODO */
#define NavFollow nav_follow(_id, _dist, _height) {}


/** Unused compat macros
 */

#define NavGlide(_start_wp, _wp) {}
#define NavVerticalAutoThrottleMode(_pitch) {}
#define NavVerticalAutoPitchMode(_throttle) {}
#define NavVerticalAltitudeMode(_alt, _pre_climb) {}
#define NavVerticalClimbMode(_climb) {}


#endif /* NAVIGATION_H */
