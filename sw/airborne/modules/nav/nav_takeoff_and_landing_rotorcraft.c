/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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

/** @file "modules/nav/nav_takeoff_and_landing_rotorcraft.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Navigation routines for takeoff and landing
 * Basic procedures for rotorcraft
 */

#include "modules/nav/nav_takeoff_and_landing.h"
#include "math/pprz_geodetic_float.h"
#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/waypoints.h"
#include "autopilot.h"

#ifndef NAV_TAKEOFF_CLIMB_SPEED
#define NAV_TAKEOFF_CLIMB_SPEED NAV_CLIMB_VSPEED
#endif

#ifndef NAV_TAKEOFF_HEIGHT
#define NAV_TAKEOFF_HEIGHT 2.f
#endif

#ifndef NAV_LANDING_DESCEND_SPEED
#define NAV_LANDING_DESCEND_SPEED NAV_DESCEND_VSPEED
#endif

#ifndef NAV_LANDING_AF_HEIGHT
#define NAV_LANDING_AF_HEIGHT 5.f
#endif

#ifndef NAV_LANDING_FLARE_HEIGHT
#define NAV_LANDING_FLARE_HEIGHT 2.f
#endif

static struct nav_takeoff takeoff;
static struct nav_landing landing;

static bool nav_takeoff_run(void);
static bool nav_land_run(void);

float nav_takeoff_direction;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_takeoff_mission(uint8_t nb UNUSED, float *params UNUSED, enum MissionRunFlag flag)
{
  if (flag == MissionInit) {
    takeoff.status = NAV_TAKEOFF_INIT;
    return nav_takeoff_from_here();
  }
  else if (flag == MissionRun) {
    return nav_takeoff_run();
  }
  return false; // not a valid case
}

static bool nav_land_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 1) {
    float td_alt = params[0];
    landing.status = NAV_LANDING_INIT;
    return nav_land_here(td_alt, 0.f);
  } else if (flag == MissionInit && nb == 2) {
    uint8_t td_id = (uint8_t)(params[0]);
    uint8_t af_id = (uint8_t)(params[1]);
    return nav_land_at_wp(td_id, af_id, 0.f);
  } else if (flag == MissionInit && nb == 5) {
    float td_alt = params[0];
    float lat = params[1];
    float lon = params[2];
    float dir = params[3];
    float dist = params[4];
    return nav_land_at_loc(td_alt, lat, lon, dir, dist, 0.f);
  } else if (flag == MissionRun) {
    return nav_land_run();
  }
  return false; // not a valid case
}

#endif

void nav_takeoff_and_landing_init(void)
{
  takeoff.status = NAV_TAKEOFF_INIT;
  takeoff.climb_id = 0;
  takeoff.timeout = false;
  landing.status = NAV_LANDING_INIT;
  landing.radius = 0.f; // no effect
  landing.td_id = 0;
  landing.af_id = 0;
  landing.timeout = false;

  nav_takeoff_direction = QFU;

#if USE_MISSION
  mission_register(nav_takeoff_mission, "TKOFF");
  mission_register(nav_land_mission, "LAND");
#endif
}

void nav_takeoff_and_landing_periodic(void)
{
  if (takeoff.timeout) {
    takeoff.status = NAV_TAKEOFF_INIT;
  }
  if (landing.timeout) {
    landing.status = NAV_LANDING_INIT;
  }

  // reset flag to true, set to false if run function is called during period
  takeoff.timeout = true;
  landing.timeout = true;
}

static bool nav_takeoff_run(void) {
  static int start_motor_counter = 0;

  switch (takeoff.status) {
    case NAV_TAKEOFF_INIT:
      takeoff.status = NAV_TAKEOFF_START_MOTOR;
      start_motor_counter = 0;
      break;
    case NAV_TAKEOFF_START_MOTOR:
      NavResurrect();
      start_motor_counter++;
      if (start_motor_counter == NAVIGATION_FREQUENCY) {
        start_motor_counter = 0;
        if (autopilot_get_motors_on()) {
          takeoff.status = NAV_TAKEOFF_CLIMB; // next step after 1s
        } else {
          takeoff.status = NAV_TAKEOFF_DONE; // failed to start motors, end now
        }
      }
      break;
    case NAV_TAKEOFF_CLIMB:
      // call vertical climb from nav/guidance
      NavGotoWaypoint(takeoff.climb_id);
      NavVerticalClimbMode(NAV_TAKEOFF_CLIMB_SPEED);
      if (stateGetPositionEnu_f()->z - takeoff.start_pos.z > NAV_TAKEOFF_HEIGHT) {
        // end when takeoff height is reached
        takeoff.status = NAV_TAKEOFF_DONE;
      }
      break;
    case NAV_TAKEOFF_DONE:
    default:
      takeoff.status = NAV_TAKEOFF_INIT;
      return false;
  }
  takeoff.timeout = false;
  return true;
}

bool nav_takeoff_from_wp(uint8_t wp_id)
{
  if (takeoff.status == NAV_TAKEOFF_INIT) {
    takeoff.climb_id = wp_id;
    waypoint_set_here_2d(wp_id);
    takeoff.climb_pos = *waypoint_get_enu_f(wp_id);
    takeoff.start_pos = *stateGetPositionEnu_f();
  }

  return nav_takeoff_run();
}

bool nav_takeoff_from_loc(float lat UNUSED, float lon UNUSED)
{
  return nav_takeoff_from_here();
}

bool nav_takeoff_from_here(void)
{
  if (takeoff.status == NAV_TAKEOFF_INIT) {
    takeoff.climb_id = 0; // use dummy hidden WP
    takeoff.start_pos = *stateGetPositionEnu_f();
    takeoff.climb_pos = takeoff.start_pos;
    waypoint_set_enu(0, &takeoff.climb_pos);
  }

  return nav_takeoff_run();
}


static bool nav_land_run(void)
{
  float dx, dy, dist2;
  switch (landing.status) {
    case NAV_LANDING_INIT:
      landing.status = NAV_LANDING_REACH_AF;
      break;
    case NAV_LANDING_REACH_AF:
      nav.nav_goto(&landing.af_pos);
      NavVerticalAltitudeMode(landing.af_pos.z, 0.f);
      if (nav.nav_approaching(&landing.af_pos, NULL, 0.f)) {
        landing.status = NAV_LANDING_DESCENT;
      }
      break;
    case NAV_LANDING_DESCENT:
      // test if glide is needed or not
      dx = landing.af_pos.x - landing.td_pos.x;
      dy = landing.af_pos.y - landing.td_pos.y;
      dist2 = dx*dx + dy*dy;
      if (dist2 > 1.f) {
        nav.nav_route(&landing.af_pos, &landing.td_pos);
        nav_glide_points(&landing.af_pos, &landing.td_pos);
      } else {
        nav.nav_goto(&landing.td_pos);
        NavVerticalClimbMode(NAV_LANDING_DESCEND_SPEED);
      }
      if (nav.nav_approaching(&landing.td_pos, &landing.af_pos, 0.f)
          || (stateGetPositionEnu_f()->z < landing.td_pos.z)) {
        landing.status = NAV_LANDING_FLARE;
      }
      break;
    case NAV_LANDING_FLARE:
      nav.nav_goto(&landing.td_pos);
      NavVerticalClimbMode(NAV_LANDING_DESCEND_SPEED);
      if (!nav_is_in_flight()) {
        landing.status = NAV_LANDING_DONE;
      }
      break;
    case NAV_LANDING_DONE:
    default:
      NavKillThrottle();
      landing.status = NAV_LANDING_INIT;
      return false;
  }
  landing.timeout = false;
  return true;
}

bool nav_land_at_wp(uint8_t td_id, uint8_t af_id, float radius UNUSED)
{
  if (landing.status == NAV_LANDING_INIT) {
    landing.td_id = td_id;
    landing.af_id = af_id;
    landing.td_pos = *waypoint_get_enu_f(td_id);
    landing.td_pos.z = landing.td_pos.z + NAV_LANDING_FLARE_HEIGHT;
    landing.af_pos = *waypoint_get_enu_f(af_id);
    landing.af_pos.z = landing.td_pos.z + NAV_LANDING_AF_HEIGHT;
  }

  return nav_land_run();
}

bool nav_land_at_loc(float td_alt, float lat, float lon, float dir, float dist, float radius UNUSED)
{
  if (landing.status == NAV_LANDING_INIT) {
    landing.td_id = 0;
    landing.af_id = 0;
    struct LlaCoor_f lla = { RadOfDeg(lat), RadOfDeg(lon), state.ned_origin_f.lla.alt + td_alt };
    enu_of_lla_point_f(&landing.td_pos, &state.ned_origin_f, &lla);
    landing.td_pos.z = landing.td_pos.z + NAV_LANDING_FLARE_HEIGHT;
    landing.af_pos.x = landing.td_pos.x + dist * sinf(RadOfDeg(dir));
    landing.af_pos.y = landing.td_pos.y + dist * cosf(RadOfDeg(dir));
    landing.af_pos.z = landing.td_pos.z + NAV_LANDING_AF_HEIGHT;
  }

  return nav_land_run();
}

bool nav_land_here(float td_alt, float radius UNUSED)
{
  if (landing.status == NAV_LANDING_INIT) {
    landing.td_id = 0;
    landing.af_id = 0;
    landing.td_pos = *stateGetPositionEnu_f();
    landing.td_pos.z = td_alt + NAV_LANDING_FLARE_HEIGHT;
    landing.af_pos = landing.td_pos;
    landing.af_pos.z = landing.td_pos.z + NAV_LANDING_AF_HEIGHT;
  }

  return nav_land_run();
}

