/*
 * Copyright (C) 2008-2011  The Paparazzi Team
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/navigation.h
 *
 * Rotorcraft navigation functions.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "state.h"
#include "modules/nav/waypoints.h"
#include "modules/nav/common_flight_plan.h"
#include "autopilot.h"
#include "generated/airframe.h"

/** default nav_circle_radius in meters */
#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 5.f
#endif

#ifndef NAV_CLIMB_VSPEED
#define NAV_CLIMB_VSPEED 0.5f
#endif

#ifndef NAV_DESCEND_VSPEED
#define NAV_DESCEND_VSPEED -0.8f
#endif

/** minimum horizontal distance to waypoint to mark as arrived */
#ifndef ARRIVED_AT_WAYPOINT
#define ARRIVED_AT_WAYPOINT 3.0f
#endif

/** Maximum distance from HOME waypoint before going into failsafe mode */
#ifndef FAILSAFE_MODE_DISTANCE
#define FAILSAFE_MODE_DISTANCE (1.2*MAX_DIST_FROM_HOME)
#endif

/** Carrot distance during navigation */
#ifndef NAV_CARROT_DIST
#define NAV_CARROT_DIST 12
#endif

/** default approaching_time for a wp */
#ifndef CARROT
#define CARROT 0
#endif

/** default navigation frequency */
#ifndef NAVIGATION_FREQUENCY
#if PERIODIC_FREQUENCY == 512
#define NAVIGATION_FREQUENCY 16
#else // if not 512, assume a multiple of 20 (e.g. 200, 500, 1000, ...)
#define NAVIGATION_FREQUENCY 20
#endif
#endif

/** Nav modes
 *  these modes correspond to the flight plan instructions used
 *  to set the high level navigation */
#define NAV_HORIZONTAL_MODE_WAYPOINT  0
#define NAV_HORIZONTAL_MODE_ROUTE     1
#define NAV_HORIZONTAL_MODE_CIRCLE    2
#define NAV_HORIZONTAL_MODE_ATTITUDE  3
#define NAV_HORIZONTAL_MODE_MANUAL    4
#define NAV_HORIZONTAL_MODE_GUIDED    5

#define NAV_VERTICAL_MODE_MANUAL      0
#define NAV_VERTICAL_MODE_CLIMB       1
#define NAV_VERTICAL_MODE_ALT         2
#define NAV_VERTICAL_MODE_GUIDED      3

/** Nav setpoint modes
 *  these modes correspond to submodes defined by navigation routines
 *  to tell which setpoint should be considered */
#define NAV_SETPOINT_MODE_POS         0
#define NAV_SETPOINT_MODE_SPEED       1
#define NAV_SETPOINT_MODE_ACCEL       2
#define NAV_SETPOINT_MODE_ATTITUDE    3 // attitude defined by roll, pitch and heading
#define NAV_SETPOINT_MODE_QUAT        4 // attitude defined by unit quaternion
#define NAV_SETPOINT_MODE_RATE        5
#define NAV_SETPOINT_MODE_MANUAL      6

typedef void (*navigation_stage_init)(void);
typedef void (*navigation_goto)(struct EnuCoor_f *wp);
typedef void (*navigation_route)(struct EnuCoor_f *wp_start, struct EnuCoor_f *wp_end);
typedef bool (*navigation_approaching)(struct EnuCoor_f *wp_to, struct EnuCoor_f *wp_from, float approaching_time);
typedef void (*navigation_circle)(struct EnuCoor_f *wp_center, float radius);
typedef void (*navigation_oval_init)(void);
typedef void (*navigation_oval)(struct EnuCoor_f *wp1, struct EnuCoor_f *wp2, float radius);


/** General Navigation structure
 */
struct RotorcraftNavigation {
  // mode
  uint8_t horizontal_mode;  // nav horizontal mode
  uint8_t vertical_mode;    // nav vertical mode
  uint8_t setpoint_mode;    // nav setpoint mode

  // commands
  struct EnuCoor_f target;  ///< final target position (in meters)
  struct EnuCoor_f carrot;  ///< carrot position (also used for GCS display)
  struct EnuCoor_f speed;   ///< speed setpoint (in m/s)
  struct EnuCoor_f accel;   ///< accel setpoint (in m/s)
  uint32_t throttle;        ///< throttle command (in pprz_t)
  int32_t cmd_roll;         ///< roll command (in pprz_t)
  int32_t cmd_pitch;        ///< pitch command (in pprz_t)
  int32_t cmd_yaw;          ///< yaw command (in pprz_t)
  float roll;               ///< roll angle (in radians)
  float pitch;              ///< pitch angle (in radians)
  float heading;            ///< heading setpoint (in radians)
  struct FloatQuat quat;    ///< quaternion setpoint
  struct FloatRates rates;  ///< rates setpoint (in rad/s)
  float radius;             ///< radius setpoint (in meters)
  float climb;              ///< climb setpoint (in m/s)
  float fp_altitude;        ///< altitude setpoint from flight plan (in meters)
  float nav_altitude;       ///< current altitude setpoint (in meters): might differ from fp_altitude depending on altitude shift from operator

  // misc
  float dist2_to_home;        ///< squared distance to home waypoint
  bool too_far_from_home;     ///< too_far flag
  float failsafe_mode_dist2;  ///< maximum squared distance to home wp before going to failsafe mode
  float climb_vspeed;         ///< climb speed setting, mostly used in flight plans
  float descend_vspeed;       ///< descend speed setting, mostly used in flight plans

  // pointers to basic nav functions
  navigation_stage_init nav_stage_init;
  navigation_goto nav_goto;
  navigation_route nav_route;
  navigation_approaching nav_approaching;
  navigation_circle nav_circle;
  navigation_oval_init nav_oval_init;
  navigation_oval nav_oval;
};

extern struct RotorcraftNavigation nav;

/** Registering functions
 */
extern void nav_register_stage_init(navigation_stage_init nav_stage_init);
extern void nav_register_goto_wp(navigation_goto nav_goto, navigation_route nav_route, navigation_approaching nav_approaching);
extern void nav_register_circle(navigation_circle nav_circle);
extern void nav_register_oval(navigation_oval_init _nav_oval_init, navigation_oval nav_oval);
// TODO: eight, survey


// flight altitude setting
extern float flight_altitude; // hmsl flight altitude in meters


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

static float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = RadOfDeg(lat_now);
	const double lat_next_rad = RadOfDeg(lat_next);

	const double cos_lat_next = cos(lat_next_rad);
	const double d_lon = RadOfDeg(lon_next - lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */

	const float y = (sin(d_lon) * cos_lat_next);
	const float x = (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next * cos(d_lon));

  float a = atan2f(y, x);
  FLOAT_ANGLE_NORMALIZE(a);

	return a;
}

static void
get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n, float *v_e)
{
  double CONSTANTS_RADIUS_OF_EARTH = 6371000.0;

	const double lat_now_rad = RadOfDeg(lat_now);
	const double lat_next_rad = RadOfDeg(lat_next);
	const double d_lon = RadOfDeg(lon_next) - RadOfDeg(lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = (CONSTANTS_RADIUS_OF_EARTH * (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon)));
	*v_e = (CONSTANTS_RADIUS_OF_EARTH * sin(d_lon) * cos(lat_next_rad));
}


extern void nav_init(void);
extern void nav_run(void);
extern void nav_parse_BLOCK(uint8_t *buf);
extern void nav_parse_MOVE_WP(uint8_t *buf);

extern float get_dist2_to_waypoint(uint8_t wp_id);
extern float get_dist2_to_point(struct EnuCoor_f *p);
extern void compute_dist2_to_home(void);
extern void nav_home(void);
extern void nav_set_manual(int32_t roll, int32_t pitch, int32_t yaw);

extern void nav_reset_reference(void) __attribute__((unused));
extern void nav_reset_alt(void) __attribute__((unused));
extern void nav_periodic_task(void);

extern bool nav_is_in_flight(void);

extern void nav_glide_points(struct EnuCoor_f *start_point, struct EnuCoor_f *end_point);

/** heading utility functions */
extern void nav_set_heading_rad(float rad);
extern void nav_set_heading_deg(float deg);
extern void nav_set_heading_towards(float x, float y);
extern void nav_set_heading_towards_waypoint(uint8_t wp);
extern void nav_set_heading_towards_target(void);
extern void nav_set_heading_current(void);

extern void nav_set_failsafe(void);

/* ground detection */
extern bool nav_detect_ground(void);
#define NavStartDetectGround() ({ autopilot.detect_ground_once = true; false; })
#define NavDetectGround() nav_detect_ground()

/* switching motors on/off */
static inline void NavKillThrottle(void)
{
  if (autopilot_get_mode() == AP_MODE_NAV) { autopilot_set_motors_on(FALSE); }
}
static inline void NavResurrect(void)
{
  if (autopilot_get_mode() == AP_MODE_NAV) { autopilot_set_motors_on(TRUE); }
}


#define NavSetManual nav_set_manual
#define NavSetFailsafe nav_set_failsafe

#define NavSetGroundReferenceHere nav_reset_reference
#define NavSetAltitudeReferenceHere nav_reset_alt

#define NavSetWaypointHere waypoint_set_here_2d
#define NavCopyWaypoint waypoint_copy
#define NavCopyWaypointPositionOnly waypoint_position_copy

/** Check the time spent in a radius of 'ARRIVED_AT_WAYPOINT' around a wp  */
bool nav_check_wp_time(struct EnuCoor_f *wp, uint16_t stay_time);
#define NavCheckWaypointTime(wp, time) nav_check_wp_time(&waypoints[wp].enu_f, time)


/***********************************************************
 * macros used by flight plan to set different modes
 **********************************************************/

#define NavAttitude(_roll) {                            \
    nav.horizontal_mode = NAV_HORIZONTAL_MODE_ATTITUDE; \
    nav.setpoint_mode = NAV_SETPOINT_MODE_ATTITUDE;     \
    nav.roll = _roll;                                   \
  }

/** Set the climb control to auto-throttle with the specified pitch
    pre-command */
#define NavVerticalAutoThrottleMode(_pitch) {   \
    nav.pitch = _pitch;                         \
  }

/** Set the vertical mode to altitude control with the specified altitude
    setpoint and climb pre-command. */
#define NavVerticalAltitudeMode(_alt, _pre_climb) { \
    nav.vertical_mode = NAV_VERTICAL_MODE_ALT;      \
    nav.fp_altitude = _alt;                         \
  }

/** Set the vertical mode to climb control with the specified climb setpoint */
#define NavVerticalClimbMode(_climb) {            \
    nav.vertical_mode = NAV_VERTICAL_MODE_CLIMB;  \
    nav.climb = _climb;                           \
    nav.speed.z = _climb;                         \
  }

/** Set the vertical mode to fixed throttle with the specified setpoint */
#define NavVerticalThrottleMode(_throttle) {      \
    nav.vertical_mode = NAV_VERTICAL_MODE_MANUAL; \
    nav.throttle = _throttle;                     \
  }

/** Set the heading of the rotorcraft, nothing else */
#define NavHeading nav_set_heading_rad


/***********************************************************
 * built in navigation routines
 **********************************************************/

/*********** Navigation to  waypoint *************************************/
static inline void NavGotoWaypoint(uint8_t wp)
{
  // waypoint_get_lla
  struct LlaCoor_i * _wp = waypoint_get_lla(wp);
  struct LlaCoor_f _wp_f;
  LLA_FLOAT_OF_BFP(_wp_f, *_wp);
  // get ltp
  struct LlaCoor_i * lla_curpos;
  lla_curpos = stateGetPositionLla_i();
  struct LlaCoor_f * lla_curpos_f;
  lla_curpos_f = stateGetPositionLla_f();
  // ltp_def_from_lla_i(&ltp_curpos, lla_curpos);
  struct LtpDef_f *ltp_curpos;
  ltp_curpos = stateGetLtp_f();
  // lla to ltp (local ned)
  // Convert lla to ned (in cm, no pos_frac!)
  struct NedCoor_f waypoint_ltp;
  ned_of_lla_point_f(&waypoint_ltp, ltp_curpos, &_wp_f);
  // separate altitude
  waypoint_ltp.z = -_wp_f.alt;

  struct EnuCoor_f waypoint_ltp_enu;
  waypoint_ltp_enu.x = waypoint_ltp.y;
  waypoint_ltp_enu.y = waypoint_ltp.x;
  waypoint_ltp_enu.z = -waypoint_ltp.z;

  float v_n, v_e;
  get_vector_to_next_waypoint(((double) lla_curpos->lat)/1.0e7, ((double) lla_curpos->lon)/1.0e7, ((double) _wp->lat)/1.0e7, ((double) _wp->lon)/1.0e7, &v_n, &v_e);

  float bearing = get_bearing_to_next_waypoint(((double) lla_curpos->lat)/1.0e7, ((double) lla_curpos->lon)/1.0e7, ((double) _wp->lat)/1.0e7, ((double) _wp->lon)/1.0e7);

  RunOnceEvery(30, printf("la lon wp: %f, %f, %f\n", _wp_f.lat, _wp_f.lon, _wp_f.alt));
  RunOnceEvery(30, printf("ltp : %f, %f, %f\n", ltp_curpos->lla.lat, ltp_curpos->lla.lon, ltp_curpos->lla.alt));
  RunOnceEvery(30, printf("pos sp enu: %f, %f, %f\n", waypoint_ltp_enu.x, waypoint_ltp_enu.y, waypoint_ltp_enu.z));
  RunOnceEvery(30, printf("new func: %f, %f, %f\n", v_e, v_n, bearing));
  double lat = ((double) lla_curpos->lat)/1.0e7;
  double lon = ((double) lla_curpos->lon)/1.0e7;
  RunOnceEvery(30, printf("la lon: %f, %f\n", lat, lon));
  // double latwp = ((double) _wp->lat)/1.0e7;
  // double lonwp = ((double) _wp->lon)/1.0e7;
  RunOnceEvery(30, printf("la lon : %f, %f\n", lla_curpos_f->lat, lla_curpos_f->lon));

  if (_wp != NULL) {
    // nav.nav_goto(_wp);
    nav.nav_goto(&waypoint_ltp_enu);
  }
}

/*********** Navigation along a line *************************************/
static inline void NavSegment(uint8_t wp_start, uint8_t wp_end)
{
  struct EnuCoor_f * _wp_start = waypoint_get_enu_f(wp_start);
  struct EnuCoor_f * _wp_end = waypoint_get_enu_f(wp_end);
  if (_wp_start != NULL && _wp_end != NULL) {
    nav.nav_route(_wp_start, _wp_end);
  }
}

static inline bool NavApproaching(uint8_t wp, float approaching_time)
{
  struct EnuCoor_f * _wp = waypoint_get_enu_f(wp);
  if (_wp != NULL) {
    return nav.nav_approaching(_wp, NULL, approaching_time);
  }
  return true; // not valid, end now
}

static inline bool NavApproachingFrom(uint8_t to, uint8_t from, float approaching_time)
{
  struct EnuCoor_f * _to = waypoint_get_enu_f(to);
  struct EnuCoor_f * _from = waypoint_get_enu_f(from);
  if (_to != NULL && _from != NULL) {
    return nav.nav_approaching(_to, _from, approaching_time);
  }
  return true; // not valid, end now
}

/*********** Navigation on a circle **************************************/
static inline void NavCircleWaypoint(uint8_t wp_center, float radius)
{
  struct EnuCoor_f * _wp_center = waypoint_get_enu_f(wp_center);
  if (_wp_center != NULL) {
    nav.nav_circle(_wp_center, radius);
  }
}


/*********** Navigation along an oval *************************************/
static inline void nav_oval_init(void)
{
  nav.nav_oval_init();
}

static inline void Oval(uint8_t wp1, uint8_t wp2, float radius)
{
  struct EnuCoor_f * _wp1 = waypoint_get_enu_f(wp1);
  struct EnuCoor_f * _wp2 = waypoint_get_enu_f(wp2);
  if (_wp1 != NULL && _wp2 != NULL) {
    nav.nav_oval(_wp1, _wp2, radius);
  }
}


/** Nav glide routine */

static inline void NavGlide(uint8_t wp_start, uint8_t wp_end)
{
  struct EnuCoor_f * _wp_start = waypoint_get_enu_f(wp_start);
  struct EnuCoor_f * _wp_end = waypoint_get_enu_f(wp_end);
  if (_wp_start != NULL && _wp_end != NULL) {
    nav_glide_points(_wp_start, _wp_end);
  }
}


/***********************************************************
 * settings handlers
 **********************************************************/
#define nav_IncreaseShift(x) {}
#define nav_SetNavRadius(x) {}
#define navigation_SetFlightAltitude(x) {                         \
    flight_altitude = x;                                          \
    nav.nav_altitude = flight_altitude - state.ned_origin_f.hmsl; \
  }

/** Unused compat macros
 */

#define NavVerticalAutoPitchMode(_throttle) {}

/* follow another aircraft not implemented with this function
 * see dedicated nav modules
 */
#define NavFollow(_i, _d, _h) {}

#endif /* NAVIGATION_H */
