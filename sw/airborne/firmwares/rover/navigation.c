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
 * @file firmwares/rover/navigation.c
 *
 * Rover navigation functions.
 */


#define NAV_C

#include "firmwares/rover/navigation.h"

#include "pprz_debug.h"
#include "subsystems/gps.h" // needed by auto_nav from the flight plan
#include "subsystems/ins.h"
#include "state.h"

#include "autopilot.h"
#include "generated/modules.h"
#include "generated/flight_plan.h"

#include "math/pprz_algebra_int.h"

#include "subsystems/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"



struct RoverNavigation nav;

uint8_t last_wp __attribute__((unused));

const float max_dist_from_home = MAX_DIST_FROM_HOME;
const float max_dist2_from_home = MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME;
float failsafe_mode_dist2 = FAILSAFE_MODE_DISTANCE * FAILSAFE_MODE_DISTANCE;


void set_exception_flag(uint8_t flag_num)
{
  nav.exception_flag[flag_num] = true;
}


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_wp_moved(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t i;
  i++;
  if (i >= nb_waypoint) { i = 0; }
  pprz_msg_send_WP_MOVED_ENU(trans, dev, AC_ID,
                             &i,
                             &(waypoints[i].enu_i.x),
                             &(waypoints[i].enu_i.y),
                             &(waypoints[i].enu_i.z));
}
#endif

void nav_init(void)
{
  waypoints_init();

  nav_block = 0;
  nav_stage = 0;

  nav.mode = NAV_MODE_WAYPOINT;

  VECT3_COPY(nav.target, waypoints[WP_HOME].enu_f);
  VECT3_COPY(nav.carrot, waypoints[WP_HOME].enu_f);
  nav.heading = 0.f;
  nav.radius = DEFAULT_CIRCLE_RADIUS;
  nav.speed = 0.f;
  nav.turn = 0.f;
  nav.shift = 0.f;

  nav.too_far_from_home = false;
  nav.dist2_to_home = 0.f;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WP_MOVED, send_wp_moved);
#endif

  // generated init function
  auto_nav_init();
}


void nav_run(void)
{
  VECT2_COPY(nav.carrot, nav.target);
}


bool nav_check_wp_time(struct EnuCoor_f *wp, float stay_time)
{
  (void) wp;
  (void) stay_time;
  return true;
//  uint16_t time_at_wp;
//  float dist_to_point;
//  static uint16_t wp_entry_time = 0;
//  static bool wp_reached = false;
//  static struct EnuCoor_i wp_last = { 0, 0, 0 };
//  struct Int32Vect2 diff;
//
//  if ((wp_last.x != wp->x) || (wp_last.y != wp->y)) {
//    wp_reached = false;
//    wp_last = *wp;
//  }
//
//  VECT2_DIFF(diff, *wp, *stateGetPositionEnu_i());
//  struct FloatVect2 diff_f = {POS_FLOAT_OF_BFP(diff.x), POS_FLOAT_OF_BFP(diff.y)};
//  dist_to_point = float_vect2_norm(&diff_f);
//  if (dist_to_point < ARRIVED_AT_WAYPOINT) {
//    if (!wp_reached) {
//      wp_reached = true;
//      wp_entry_time = autopilot.flight_time;
//      time_at_wp = 0;
//    } else {
//      time_at_wp = autopilot.flight_time - wp_entry_time;
//    }
//  } else {
//    time_at_wp = 0;
//    wp_reached = false;
//  }
//  if (time_at_wp > stay_time) {
//    INT_VECT3_ZERO(wp_last);
//    return true;
//  }
//  return false;
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
  VECT3_COPY(nav.last_pos, *stateGetPositionEnu_f());
  stage_time = 0;
  //nav_circle_radians = 0; FIXME
}

void nav_periodic_task(void)
{
  RunOnceEvery(NAV_FREQ, { stage_time++;  block_time++; });

  //nav.dist2_to_wp = 0; FIXME

  /* from flight_plan.h */
  auto_nav();

  /* run carrot loop */
  nav_run();
}

bool nav_is_in_flight(void)
{
  return autopilot_in_flight();
}

/** Home mode navigation */
void nav_home(void)
{
  nav.mode = NAV_MODE_WAYPOINT;
  VECT3_COPY(nav.target, waypoints[WP_HOME].enu_f);

  //nav.dist2_to_wp = nav.dist2_to_home; FIXME

  /* run carrot loop */
  nav_run();
}

/** Returns squared horizontal distance to given point */
float get_dist2_to_point(struct EnuCoor_f *p)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct FloatVect2 pos_diff = {
    .x = p->x - pos->x,
    .y = p->y - pos->y
  };
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
    nav.heading = atan2f(pos_diff.x, pos_diff.y);
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
#ifdef AP_MODE_FAILSAFE
  autopilot_set_mode(AP_MODE_FAILSAFE);
#endif
}


/** Register functions
 */

void nav_register_goto_wp(nav_rover_goto nav_goto, nav_rover_route nav_route, nav_rover_approaching nav_approaching)
{
  nav.nav_goto = nav_goto;
  nav.nav_route = nav_route;
  nav.nav_approaching = nav_approaching;
}

void nav_register_circle(nav_rover_circle nav_circle)
{
  nav.nav_circle = nav_circle;
}

void nav_register_oval(nav_rover_oval_init nav_oval_init, nav_rover_oval nav_oval)
{
  nav.nav_oval_init = nav_oval_init;
  nav.nav_oval = nav_oval;
}


