/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file firmwares/rotorcraft/navigation.c
 *
 * Rotorcraft navigation functions.
 */


#define NAV_C

#include "firmwares/rotorcraft/navigation.h"

#include "pprz_debug.h"
#include "state.h"
#include "autopilot.h"
#include "generated/modules.h"
#include "generated/flight_plan.h"
#include "modules/ins/ins.h"

/* for default GUIDANCE_H_USE_REF */
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "math/pprz_algebra_int.h"

#include "modules/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"

PRINT_CONFIG_VAR(NAVIGATION_FREQUENCY)

struct RotorcraftNavigation nav;

const float max_dist_from_home = MAX_DIST_FROM_HOME;
const float max_dist2_from_home = MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME;

float flight_altitude;

/** Empty navigation functions set at init
 */
static void empty_stage_init(void) {}
static void empty_goto(struct EnuCoor_f *wp UNUSED) {}
static void empty_route(struct EnuCoor_f *wp_start UNUSED, struct EnuCoor_f *wp_end UNUSED) {}
static bool empty_approaching(struct EnuCoor_f *wp_to UNUSED, struct EnuCoor_f *wp_from UNUSED, float approaching_time UNUSED) { return true; }
static void empty_circle(struct EnuCoor_f *wp_center UNUSED, float radius UNUSED) {}
static void empty_oval_init(void) {}
static void empty_oval(struct EnuCoor_f *wp1 UNUSED, struct EnuCoor_f *wp2 UNUSED, float radius UNUSED) {};

static inline void nav_set_altitude(void);

void nav_init(void)
{
  waypoints_init();

  nav_block = 0;
  nav_stage = 0;

  nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
  nav.vertical_mode = NAV_VERTICAL_MODE_ALT;
  nav.setpoint_mode = NAV_SETPOINT_MODE_POS;

  VECT3_COPY(nav.target, waypoints[WP_HOME].enu_f);
  VECT3_COPY(nav.carrot, waypoints[WP_HOME].enu_f);
  FLOAT_VECT3_ZERO(nav.speed);
  FLOAT_VECT3_ZERO(nav.accel);
  float_quat_identity(&nav.quat);
  FLOAT_RATES_ZERO(nav.rates);

  nav.throttle = 0;
  nav.cmd_roll = 0;
  nav.cmd_pitch = 0;
  nav.cmd_yaw = 0;
  nav.roll = 0.f;
  nav.pitch = 0.f;
  nav.heading = 0.f;
  nav.radius = DEFAULT_CIRCLE_RADIUS;
  nav.climb = 0.f;
  nav.fp_altitude = SECURITY_HEIGHT;
  nav.nav_altitude = SECURITY_HEIGHT;
  flight_altitude = SECURITY_ALT;

  nav.too_far_from_home = false;
  nav.failsafe_mode_dist2 = FAILSAFE_MODE_DISTANCE * FAILSAFE_MODE_DISTANCE;
  nav.dist2_to_home = 0.f;
  nav.climb_vspeed = NAV_CLIMB_VSPEED;
  nav.descend_vspeed = NAV_DESCEND_VSPEED;

  nav.nav_stage_init = empty_stage_init;
  nav.nav_goto = empty_goto;
  nav.nav_route = empty_route;
  nav.nav_approaching = empty_approaching;
  nav.nav_circle = empty_circle;
  nav.nav_oval_init = empty_oval_init;
  nav.nav_oval = empty_oval;

  // generated init function
  auto_nav_init();
}

void nav_parse_BLOCK(uint8_t *buf)
{
  if (DL_BLOCK_ac_id(buf) != AC_ID) { return; }
  nav_goto_block(DL_BLOCK_block_id(buf));
}

void nav_parse_MOVE_WP(uint8_t *buf)
{
  uint8_t ac_id = DL_MOVE_WP_ac_id(buf);
  if (ac_id != AC_ID) { return; }
  if (stateIsLocalCoordinateValid()) {
    uint8_t wp_id = DL_MOVE_WP_wp_id(buf);
    struct LlaCoor_i lla;
    lla.lat = DL_MOVE_WP_lat(buf);
    lla.lon = DL_MOVE_WP_lon(buf);
    /* WP_alt from message is alt above MSL in mm
     * lla.alt is above ellipsoid in mm
     */
    lla.alt = DL_MOVE_WP_alt(buf) - state.ned_origin_i.hmsl +
      state.ned_origin_i.lla.alt;
    waypoint_move_lla(wp_id, &lla);
  }
}

#ifndef CLOSE_TO_WAYPOINT
#define CLOSE_TO_WAYPOINT 15.f
#endif

static inline void UNUSED nav_advance_carrot(void)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  /* compute a vector to the waypoint */
  struct FloatVect2 path_to_waypoint;
  VECT2_DIFF(path_to_waypoint, nav.target, *pos);

  /* saturate it */
  VECT2_STRIM(path_to_waypoint, -150.f, 150.f);

  float dist_to_waypoint = float_vect2_norm(&path_to_waypoint);

  if (dist_to_waypoint < CLOSE_TO_WAYPOINT) {
    VECT2_COPY(nav.carrot, nav.target);
  } else {
    struct Int32Vect2 path_to_carrot;
    VECT2_SMUL(path_to_carrot, path_to_waypoint, NAV_CARROT_DIST);
    VECT2_SDIV(path_to_carrot, path_to_carrot, dist_to_waypoint);
    VECT2_SUM(nav.carrot, path_to_carrot, *pos);
  }
}

void nav_run(void)
{

#if GUIDANCE_H_USE_REF
  // if GUIDANCE_H_USE_REF, CARROT_DIST is not used
  VECT2_COPY(nav.carrot, nav.target);
#else
  nav_advance_carrot();
#endif

  // update altitude setpoint if needed
  nav_set_altitude();
}


bool nav_check_wp_time(struct EnuCoor_f *wp, uint16_t stay_time)
{
  uint16_t time_at_wp;
  float dist_to_point;
  static uint16_t wp_entry_time = 0;
  static bool wp_reached = false;
  static struct EnuCoor_i wp_last = { 0, 0, 0 };
  struct EnuCoor_i wp_i;
  struct FloatVect2 diff;

  ENU_BFP_OF_REAL(wp_i, *wp);
  if ((wp_last.x != wp_i.x) || (wp_last.y != wp_i.y)) {
    wp_reached = false;
    wp_last = wp_i;
  }

  VECT2_DIFF(diff, *wp, *stateGetPositionEnu_f());
  dist_to_point = float_vect2_norm(&diff);
  if (dist_to_point < ARRIVED_AT_WAYPOINT) {
    if (!wp_reached) {
      wp_reached = true;
      wp_entry_time = autopilot.flight_time;
      time_at_wp = 0;
    } else {
      time_at_wp = autopilot.flight_time - wp_entry_time;
    }
  } else {
    time_at_wp = 0;
    wp_reached = false;
  }
  if (time_at_wp > stay_time) {
    INT_VECT3_ZERO(wp_last);
    return true;
  }
  return false;
}

static inline void nav_set_altitude(void)
{
  static float last_alt = 0.f;
  // if the fp_altitude setpoint change is large enough, set this alt as the new reference
  // otherwise, don't change nav_altitude (whose value can be changed by the operator
  // through the flight_altitude setting)
  // nav_altitude is the value that is used by guidance as a setpoint when flying in
  // altitude mode
  if (fabsf(nav.fp_altitude - last_alt) > 0.2f) {
    nav.nav_altitude = nav.fp_altitude;
    flight_altitude = nav.nav_altitude + state.ned_origin_f.hmsl;
    last_alt = nav.fp_altitude;
  }
}


/** Reset the geographic reference to the current GPS fix */
void nav_reset_reference(void)
{
  ins_reset_local_origin();
  /* update local ENU coordinates of global waypoints */
  waypoints_localize_all();
}

void nav_reset_alt(void)
{
  ins_reset_altitude_ref();
  waypoints_localize_all();
}

void nav_init_stage(void)
{
  stage_time = 0;
  if (nav.nav_stage_init) {
    nav.nav_stage_init();
  }
}

#include <stdio.h>
void nav_periodic_task(void)
{
  RunOnceEvery(NAVIGATION_FREQUENCY, { stage_time++;  block_time++; });

  /* from flight_plan.h */
  auto_nav();

  /* run carrot loop */
  nav_run();
}

bool nav_detect_ground(void)
{
  if (!autopilot.ground_detected) { return false; }
  autopilot.ground_detected = false;
  return true;
}

bool nav_is_in_flight(void)
{
  return autopilot_in_flight();
}

void nav_glide_points(struct EnuCoor_f *start_point, struct EnuCoor_f *end_point)
{
  struct FloatVect2 wp_diff, pos_diff;
  VECT2_DIFF(wp_diff, *end_point, *start_point);
  VECT2_DIFF(pos_diff, *stateGetPositionEnu_f(), *start_point);
  float length2 = Max(float_vect2_norm2(&wp_diff), 0.1f);
  float progress = (pos_diff.x * wp_diff.x + pos_diff.y * wp_diff.y) / length2;
  float alt = start_point->z + (end_point->z - start_point->z) * progress;
  NavVerticalAltitudeMode(alt, 0);
}

/** Home mode navigation */
void nav_home(void)
{
  nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
  nav.setpoint_mode = NAV_SETPOINT_MODE_POS;
  VECT3_COPY(nav.target, waypoints[WP_HOME].enu_f);

  nav.vertical_mode = NAV_VERTICAL_MODE_ALT;
  nav.nav_altitude = waypoint_get_alt(WP_HOME);

  /* run carrot loop */
  nav_run();
}

/** Set manual roll, pitch and yaw without stabilization
 *
 * @param[in] roll command in pprz scale (int32_t)
 * @param[in] pitch command in pprz scale (int32_t)
 * @param[in] yaw command in pprz scale (int32_t)
 *
 * This function allows to directly set commands from the flight plan,
 * if in nav_manual mode.
 * This is for instance useful for helicopters during the spinup
 */
void nav_set_manual(int32_t roll, int32_t pitch, int32_t yaw)
{
  nav.horizontal_mode = NAV_HORIZONTAL_MODE_MANUAL;
  nav.setpoint_mode = NAV_SETPOINT_MODE_MANUAL;
  nav.cmd_roll = roll;
  nav.cmd_pitch = pitch;
  nav.cmd_yaw = yaw;
}

/** Returns squared horizontal distance to given point */
float get_dist2_to_point(struct EnuCoor_f *p)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct FloatVect2 pos_diff;
  pos_diff.x = p->x - pos->x;
  pos_diff.y = p->y - pos->y;
  return pos_diff.x * pos_diff.x + pos_diff.y * pos_diff.y;
}

/** Returns squared horizontal distance to given waypoint */
float get_dist2_to_waypoint(uint8_t wp_id)
{
  return get_dist2_to_point(&waypoints[wp_id].enu_f);
}

/** Computes squared distance to the HOME waypoint potentially sets
 * #too_far_from_home
 */
void compute_dist2_to_home(void)
{
  nav.dist2_to_home = get_dist2_to_waypoint(WP_HOME);
  nav.too_far_from_home = nav.dist2_to_home > max_dist2_from_home;
#ifdef InGeofenceSector
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  nav.too_far_from_home = nav.too_far_from_home || !(InGeofenceSector(pos->x, pos->y));
#endif
}

/** Set nav_heading in radians. */
void nav_set_heading_rad(float rad)
{
  nav.heading = rad;
  NormCourseRad(nav.heading);
}

/** Set nav_heading in degrees. */
void nav_set_heading_deg(float deg)
{
  nav_set_heading_rad(RadOfDeg(deg));
}

/** Set heading to point towards x,y position in local coordinates */
void nav_set_heading_towards(float x, float y)
{
  struct FloatVect2 target = {x, y};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  // don't change heading if closer than 0.5m to target
  if (VECT2_NORM2(pos_diff) > 0.25f) {
    float heading_f = atan2f(pos_diff.x, pos_diff.y);
    nav.heading = heading_f;
  }
}

/** Set heading in the direction of a waypoint */
void nav_set_heading_towards_waypoint(uint8_t wp)
{
  nav_set_heading_towards(WaypointX(wp), WaypointY(wp));
}

/** Set heading in the direction of the target*/
void nav_set_heading_towards_target(void)
{
  nav_set_heading_towards(nav.target.x, nav.target.y);
}

/** Set heading to the current yaw angle */
void nav_set_heading_current(void)
{
  nav.heading = stateGetNedToBodyEulers_f()->psi;
}

void nav_set_failsafe(void)
{
  autopilot_set_mode(AP_MODE_FAILSAFE);
}

/** Register functions
 */
void nav_register_stage_init(navigation_stage_init nav_stage_init)
{
  nav.nav_stage_init = nav_stage_init;
}

void nav_register_goto_wp(navigation_goto nav_goto, navigation_route nav_route, navigation_approaching nav_approaching)
{
  nav.nav_goto = nav_goto;
  nav.nav_route = nav_route;
  nav.nav_approaching = nav_approaching;
}

void nav_register_circle(navigation_circle nav_circle)
{
  nav.nav_circle = nav_circle;
}

void nav_register_oval(navigation_oval_init _nav_oval_init, navigation_oval nav_oval)
{
  nav.nav_oval_init = _nav_oval_init;
  nav.nav_oval = nav_oval;
}

