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

/** @file "modules/nav/nav_takeoff_and_landing_fw.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Navigation routines for takeoff and landing
 * Basic procedures for fixed-wing
 */

#include "modules/nav/nav_takeoff_and_landing.h"
#include "math/pprz_geodetic_float.h"
#include "generated/flight_plan.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "autopilot.h"

#ifndef NAV_TAKEOFF_PITCH
#define NAV_TAKEOFF_PITCH 15.f
#endif

#ifndef NAV_TAKEOFF_THROTTLE
#define NAV_TAKEOFF_THROTTLE 1.f
#endif

#ifndef NAV_TAKEOFF_HEIGHT
#define NAV_TAKEOFF_HEIGHT 20.f
#endif

#ifndef NAV_TAKEOFF_DIST
#define NAV_TAKEOFF_DIST 200.f
#endif

#ifndef NAV_TAKEOFF_AUTO_LAUNCH
#define NAV_TAKEOFF_AUTO_LAUNCH true
#endif

#ifndef NAV_LANDING_DESCEND_SPEED
#define NAV_LANDING_DESCEND_SPEED -1.f
#endif

#ifndef NAV_LANDING_AF_HEIGHT
#define NAV_LANDING_AF_HEIGHT 30.f
#endif

#ifndef NAV_LANDING_FLARE_HEIGHT
#define NAV_LANDING_FLARE_HEIGHT 10.f
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
  if (flag == MissionInit && nb == 2) {
    float td_alt = params[0];
    float radius = params[1];
    landing.status = NAV_LANDING_INIT;
    return nav_land_here(td_alt, radius);
  } else if (flag == MissionInit && nb == 3) {
    uint8_t td_id = (uint8_t)(params[0]);
    uint8_t af_id = (uint8_t)(params[1]);
    float radius = params[2];
    return nav_land_at_wp(td_id, af_id, radius);
  } else if (flag == MissionInit && nb == 6) {
    float td_alt = params[0];
    float lat = params[1];
    float lon = params[2];
    float dir = params[3];
    float dist = params[4];
    float radius = params[5];
    return nav_land_at_loc(td_alt, lat, lon, dir, dist, radius);
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
      autopilot_set_motors_on(true);
      start_motor_counter++;
      if (start_motor_counter == NAVIGATION_FREQUENCY) {
        start_motor_counter = 0;
        if (autopilot_get_motors_on()) {
#if NAV_TAKEOFF_AUTO_LAUNCH
          autopilot.launch = true;
#endif
          takeoff.status = NAV_TAKEOFF_CLIMB; // next step after 1s
        } else {
          takeoff.status = NAV_TAKEOFF_DONE; // failed to start motors, end now
        }
      }
      break;
    case NAV_TAKEOFF_CLIMB:
      // climb towards point at specified pitch and throttle
      NavGotoPoint(takeoff.climb_pos);
      NavVerticalAutoThrottleMode(RadOfDeg(NAV_TAKEOFF_PITCH));
      NavVerticalThrottleMode(TRIM_UPPRZ(MAX_PPRZ*NAV_TAKEOFF_THROTTLE));
      if (nav_approaching_xy(takeoff.climb_pos.x, takeoff.climb_pos.y, takeoff.start_pos.x, takeoff.start_pos.y, CARROT)
          || (stateGetPositionEnu_f()->z > takeoff.start_pos.z + NAV_TAKEOFF_HEIGHT)) {
        // end when climb point or target alt is reached
        takeoff.status = NAV_TAKEOFF_DONE;
      }
      break;
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
    takeoff.climb_pos.x = WaypointX(wp_id);
    takeoff.climb_pos.y = WaypointY(wp_id);
    takeoff.climb_pos.z = WaypointAlt(wp_id) - GetAltRef();
    takeoff.start_pos = *stateGetPositionEnu_f();
  }

  return nav_takeoff_run();
}

bool nav_takeoff_from_loc(float lat, float lon)
{
  if (takeoff.status == NAV_TAKEOFF_INIT) {
    struct LlaCoor_f lla = { RadOfDeg(lat), RadOfDeg(lon), stateGetPositionLla_f()->alt + NAV_TAKEOFF_HEIGHT };
    struct UtmCoor_f utm;
    utm_of_lla_f(&utm, &lla);
    ENU_OF_UTM_DIFF(takeoff.climb_pos, utm, state.utm_origin_f);
  }
  return nav_takeoff_from_here();
}

bool nav_takeoff_from_here(void)
{
  if (takeoff.status == NAV_TAKEOFF_INIT) {
    takeoff.climb_id = 0;
    takeoff.start_pos = *stateGetPositionEnu_f();
    takeoff.climb_pos = takeoff.start_pos;
    takeoff.climb_pos.x = takeoff.start_pos.x + NAV_TAKEOFF_DIST * sinf(RadOfDeg(nav_takeoff_direction));
    takeoff.climb_pos.y = takeoff.start_pos.y + NAV_TAKEOFF_DIST * cosf(RadOfDeg(nav_takeoff_direction));
    takeoff.climb_pos.z = takeoff.start_pos.z + NAV_TAKEOFF_HEIGHT;
  }

  return nav_takeoff_run();
}


static bool nav_land_run(void)
{
  static float baseleg_out_qdr = 0.f;
  float qdr = 0.f;

  // test if glide is needed or not
  float dx = landing.td_pos.x - landing.af_pos.x;
  float dy = landing.td_pos.y - landing.af_pos.y;
  float dist = sqrtf(dx*dx + dy*dy);
  bool glide = false;
  float x_unit = 0.f;
  float y_unit = 0.f;
  if (dist > 1.f) {
    glide = true;
    x_unit = dx / dist;
    y_unit = dy / dist;
  }

  switch (landing.status) {
    case NAV_LANDING_INIT:
      if (glide) {
        // compute baseleg circle center, store in dummy WP 0
        waypoints[0].x = landing.af_pos.x + y_unit * landing.radius;
        waypoints[0].y = landing.af_pos.y - x_unit * landing.radius;
        waypoints[0].a = landing.af_pos.z + GetAltRef();
        baseleg_out_qdr = M_PI - atan2f(-y_unit, -x_unit);
        if (landing.radius < 0.f) {
          baseleg_out_qdr += M_PI;
        }
      } else {
        // turn around AF point
        waypoints[0].x = landing.af_pos.x;
        waypoints[0].y = landing.af_pos.y;
        waypoints[0].a = landing.af_pos.z + GetAltRef();
        baseleg_out_qdr = 0.f;
      }
      landing.status = NAV_LANDING_REACH_AF;
      break;
    case NAV_LANDING_REACH_AF:
      NavVerticalAutoThrottleMode(0.f);
      NavVerticalAltitudeMode(waypoints[0].a, 0.f);
      NavCircleWaypoint(0, landing.radius);
      if (NavCircleCount() > 0.25 && (fabsf(GetPosAlt() - WaypointAlt(0)) < 10.f)) {
        if (glide) {
          // if final glide, direction should be correct
          qdr = M_PI_2 - nav_circle_trigo_qdr;
          NormCourseRad(qdr);
          if (CloseRadAngles(baseleg_out_qdr, qdr)) {
            landing.status = NAV_LANDING_DESCENT;
          }
        } else {
          // no final glide, start from correct altitude
          landing.status = NAV_LANDING_DESCENT;
        }
      }
      break;
    case NAV_LANDING_DESCENT:
      if (glide) {
        nav_route_xy(landing.af_pos.x, landing.af_pos.y, landing.td_pos.x, landing.td_pos.y);
        NavVerticalAutoThrottleMode(0.f);
        nav_glide_alt(landing.af_pos.z + GetAltRef(), landing.td_pos.z + GetAltRef());
        if (nav_approaching_xy(landing.td_pos.x, landing.td_pos.y, landing.af_pos.x, landing.af_pos.y, CARROT)
            || (GetPosAlt() < landing.td_pos.z + NAV_LANDING_FLARE_HEIGHT + GetAltRef())) {
          landing.status = NAV_LANDING_FLARE;
        }
      } else {
        NavCircleWaypoint(0, landing.radius);
        NavVerticalAutoThrottleMode(0.f);
        NavVerticalClimbMode(NAV_LANDING_DESCEND_SPEED);
        if (GetPosAlt() < landing.td_pos.z + NAV_LANDING_FLARE_HEIGHT + GetAltRef()) {
          landing.status = NAV_LANDING_FLARE;
        }
      }
      break;
    case NAV_LANDING_FLARE:
      NavVerticalAutoThrottleMode(0.f);
      NavVerticalThrottleMode(0.f);
      if (glide) {
        nav_route_xy(landing.af_pos.x, landing.af_pos.y, landing.td_pos.x, landing.td_pos.y);
        if (nav_approaching_xy(landing.td_pos.x, landing.td_pos.y, landing.af_pos.x, landing.af_pos.y, 0.f)) {
          landing.status = NAV_LANDING_DONE;
        }
      } else {
        NavCircleWaypoint(0, landing.radius);
        // stay on circle, no end
      }
      break;
    case NAV_LANDING_DONE:
    default:
      landing.status = NAV_LANDING_INIT;
      return false;
  }
  landing.timeout = false;
  return true;
}

bool nav_land_at_wp(uint8_t td_id, uint8_t af_id, float radius)
{
  if (landing.status == NAV_LANDING_INIT) {
    landing.td_id = td_id;
    landing.af_id = af_id;
    landing.td_pos.x = WaypointX(td_id);
    landing.td_pos.y = WaypointY(td_id);
    landing.td_pos.z = WaypointAlt(td_id) - GetAltRef();
    landing.af_pos.x = WaypointX(af_id);
    landing.af_pos.y = WaypointY(af_id);
    landing.af_pos.z = landing.td_pos.z + NAV_LANDING_AF_HEIGHT;
    landing.radius = radius;
  }

  return nav_land_run();
}

bool nav_land_at_loc(float td_alt, float lat, float lon, float dir, float dist, float radius)
{
  if (landing.status == NAV_LANDING_INIT) {
    landing.td_id = 0;
    landing.af_id = 0;
    struct LlaCoor_f lla = { RadOfDeg(lat), RadOfDeg(lon), GetAltRef() + td_alt };
    struct UtmCoor_f utm;
    utm_of_lla_f(&utm, &lla);
    ENU_OF_UTM_DIFF(landing.td_pos, utm, state.utm_origin_f);
    landing.af_pos.x = landing.td_pos.x + dist * sinf(RadOfDeg(dir));
    landing.af_pos.y = landing.td_pos.y + dist * cosf(RadOfDeg(dir));
    landing.af_pos.z = landing.td_pos.z + NAV_LANDING_AF_HEIGHT;
    landing.radius = radius;
  }

  return nav_land_run();
}

bool nav_land_here(float td_alt, float radius)
{
  if (landing.status == NAV_LANDING_INIT) {
    landing.td_id = 0;
    landing.af_id = 0;
    landing.td_pos = *stateGetPositionEnu_f();
    landing.td_pos.z = td_alt;
    landing.af_pos = landing.td_pos;
    landing.af_pos.z = landing.td_pos.z + NAV_LANDING_AF_HEIGHT;
    landing.radius = radius;
  }

  return nav_land_run();
}


