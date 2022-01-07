/*
 * Copyright (C) 2018-2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                         Titouan Verdu <titouan.verdu@enac.fr>
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
 * @file modules/nav/nav_rosette.c
 *
 * Adaptive flower pattern for cloud exploration
 */

#include "modules/nav/nav_rosette.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "modules/core/abi.h"

#ifndef NAV_ROSETTE_RECOVER_MAX_TURN
#define NAV_ROSETTE_RECOVER_MAX_TURN 1.5f
#endif

#ifndef NAV_ROSETTE_BORDER_FILTER
#define NAV_ROSETTE_BORDER_FILTER 60.f
#endif

enum RosetteStatus {
  RSTT_ENTER,
  RSTT_CROSSING_FIRST,
  RSTT_CROSSING_START,
  RSTT_CROSSING,
  RSTT_TURNING_START,
  RSTT_TURNING,
  RSTT_RECOVER_START,
  RSTT_RECOVER_OUTSIDE
};

enum RotationDir {
  RSTT_LEFT,
  RSTT_RIGHT
};

struct NavRosette {
  enum RosetteStatus status;
  enum RotationDir rotation;
  bool inside_cloud;
  struct EnuCoor_f actual;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  struct EnuCoor_f estim_border;
  struct EnuCoor_f recover_circle;
  struct EnuCoor_f * first;
  struct FloatVect3 pos_incr;
  float direction;
  float radius;
  float radius_sign;
  float last_border_time;
  float recover_radius;
  float max_recover_radius;
};

static struct NavRosette nav_rosette;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static void update_barycenter(struct EnuCoor_f *bary, struct EnuCoor_f *border, struct EnuCoor_f *first, float alt_sp, float dt, float tau)
{
  if (first != NULL) {
    bary->x = (border->x + first->x) / 2.f;
    bary->y = (border->y + first->y) / 2.f;
    bary->z = alt_sp;
  }
  else {
    if (tau > 1e-4) {
      float alpha = dt / (dt + tau);
      bary->x = (border->x * alpha) + (bary->x * (1.f - alpha));
      bary->y = (border->y * alpha) + (bary->y * (1.f - alpha));
      bary->z = alt_sp;
    }
  }
}

static struct EnuCoor_f process_new_point_rosette(struct EnuCoor_f *position, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_rosette.rotation == RSTT_RIGHT) {
    rot_angle = -M_PI_2;
  } else {
    rot_angle = M_PI_2;
  }

  if (nav_rosette.inside_cloud == true) {
    new_point.x = nav_rosette.target.x;
    new_point.y = nav_rosette.target.y;
    new_point.z = nav_rosette.target.z;
  }
  else if (nav_rosette.inside_cloud == false) {
    new_point.x = position->x + (cosf(rot_angle + uav_direction) * nav_rosette.radius);
    new_point.y = position->y + (sinf(rot_angle + uav_direction) * nav_rosette.radius);
    new_point.z = nav_rosette.target.z;
  }

  return new_point;
}

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_rosette_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 8) {
    float start_x = params[0];
    float start_y = params[1];
    float start_z = params[2];
    int first_turn = params[3];
    float circle_radius = params[4];
    float vx = params[5];
    float vy = params[6];
    float vz = params[7];
    nav_rosette_setup(start_x, start_y, start_z, first_turn, circle_radius, vx, vy, vz);
    return true;
  }
  else if (flag == MissionUpdate && nb == 3) {
    // update target 3D position (ENU frame, above ground alt)
    float x = params[0];
    float y = params[1];
    float z = params[2];
    VECT3_ASSIGN(nav_rosette.target, x, y, z);
    return true;
  }
  else if (flag == MissionUpdate && nb == 2) {
    // update horizontal speed
    float vx = params[0];
    float vy = params[1];
    nav_rosette.pos_incr.x = vx * nav_dt;
    nav_rosette.pos_incr.y = vy * nav_dt;
    return true;
  }
  else if (flag == MissionUpdate && nb == 1) {
    // update vertical speed
    float vz = params[0];
    nav_rosette.pos_incr.z = vz * nav_dt;
    return true;
  }
  else if (flag == MissionRun) {
    return nav_rosette_run();
  }
  return false; // not a valid case
}
#endif

// ABI message

#ifndef NAV_ROSETTE_LWC_ID
#define NAV_ROSETTE_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static void lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data)
{
  if (data_type == 1 && size == 1) {
    nav_rosette.inside_cloud = (bool) data[0];
  }
}

void nav_rosette_init(void)
{
  nav_rosette.status = RSTT_ENTER;
  nav_rosette.radius = DEFAULT_CIRCLE_RADIUS;
  nav_rosette.recover_radius = DEFAULT_CIRCLE_RADIUS;
  nav_rosette.max_recover_radius = DEFAULT_CIRCLE_RADIUS;
  nav_rosette.inside_cloud = false;
  nav_rosette.last_border_time = 0.f;

  AbiBindMsgPAYLOAD_DATA(NAV_ROSETTE_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_rosette_mission, "RSTT");
#endif
}

void nav_rosette_setup(float init_x, float init_y, float init_z,
                       int turn, float desired_radius,
                       float vx, float vy, float vz)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  // increment based on speed
  VECT3_ASSIGN(nav_rosette.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);

  nav_rosette.target = start;
  nav_rosette.status = RSTT_ENTER;
  nav_rosette.inside_cloud = false;
  nav_rosette.radius = desired_radius;
  nav_rosette.first = NULL;

  if (turn == 1) {
    nav_rosette.rotation = RSTT_RIGHT;
    nav_rosette.radius_sign = 1.0f;
  } else {
    nav_rosette.rotation = RSTT_LEFT;
    nav_rosette.radius_sign = -1.0f;
  }

  nav_rosette.actual = *stateGetPositionEnu_f();
  nav_rosette.last_border_time = get_sys_time_float();
}

bool nav_rosette_run(void)
{
  float pre_climb = 0.f;
  float time = get_sys_time_float();

  NavVerticalAutoThrottleMode(0.f); /* No Pitch */

  switch (nav_rosette.status) {
    case RSTT_ENTER:
      // init stage
      nav_init_stage();
      // reach target point
      nav_route_xy(nav_rosette.actual.x, nav_rosette.actual.y, nav_rosette.target.x, nav_rosette.target.y);
      NavVerticalAltitudeMode(nav_rosette.target.z + ground_alt, pre_climb);

      if (nav_rosette.inside_cloud) {
        // found border or already inside
        nav_rosette.status = RSTT_CROSSING_FIRST;
      }
      break;
    case RSTT_CROSSING_FIRST:
      // prepare first crossing
      nav_rosette.actual = *stateGetPositionEnu_f();
      nav_rosette.last_border_time = time;
      nav_rosette.estim_border = nav_rosette.actual;
      nav_rosette.first = &nav_rosette.estim_border;
      // init stage
      nav_init_stage();
      // cross
      nav_rosette.status = RSTT_CROSSING;
      break;
    case RSTT_CROSSING_START:
      // prepare crossing
      nav_rosette.actual = *stateGetPositionEnu_f();
      update_barycenter(&nav_rosette.target, &nav_rosette.actual, nav_rosette.first,
          nav_rosette.target.z, time - nav_rosette.last_border_time, NAV_ROSETTE_BORDER_FILTER);
      nav_rosette.last_border_time = time;
      nav_rosette.estim_border = nav_rosette.actual;
      // init stage
      nav_init_stage();
      // cross
      nav_rosette.status = RSTT_CROSSING;
      break;
    case RSTT_CROSSING:
      // cross towards target
      pre_climb = nav_rosette.pos_incr.z / nav_dt;
      nav_route_xy(nav_rosette.actual.x, nav_rosette.actual.y, nav_rosette.target.x, nav_rosette.target.y);
      NavVerticalAltitudeMode(nav_rosette.target.z + ground_alt, pre_climb);

      if (!nav_rosette.inside_cloud) {
        // found a border, starting turning back inside
        // note that you don't need a recover as you always expect to go out after some time
        nav_rosette.status = RSTT_TURNING_START;
      }
      break;
    case RSTT_TURNING_START:
      // update target
      nav_rosette.actual = *stateGetPositionEnu_f();
      update_barycenter(&nav_rosette.target, &nav_rosette.actual, nav_rosette.first,
          nav_rosette.target.z, time - nav_rosette.last_border_time, NAV_ROSETTE_BORDER_FILTER);
      nav_rosette.last_border_time = time;
      nav_rosette.estim_border = nav_rosette.actual;
      nav_rosette.first = NULL;
      // prepare next circle
      nav_rosette.direction = change_rep(stateGetHorizontalSpeedDir_f());
      nav_rosette.circle = process_new_point_rosette(&nav_rosette.actual, nav_rosette.direction);
      // init stage
      nav_init_stage();
      // turn
      nav_rosette.status = RSTT_TURNING;
      break;
    case RSTT_TURNING:
      // update circle center
      VECT3_ADD(nav_rosette.circle, nav_rosette.pos_incr);
      pre_climb = nav_rosette.pos_incr.z / nav_dt;
      nav_circle_XY(nav_rosette.circle.x, nav_rosette.circle.y , nav_rosette.radius_sign * nav_rosette.radius);
      NavVerticalAltitudeMode(nav_rosette.circle.z + ground_alt, pre_climb);

      if (nav_rosette.inside_cloud) {
        // found a border, cross again
        nav_rosette.status = RSTT_CROSSING_START;
      }
      else if (NavCircleCountNoRewind() > NAV_ROSETTE_RECOVER_MAX_TURN) {
        // most likely lost outside
        nav_rosette.status = RSTT_RECOVER_START;
      }
      break;
    case RSTT_RECOVER_START:
      // prepare recovery circle
      nav_rosette.recover_circle = nav_rosette.target;
      nav_rosette.actual = *stateGetPositionEnu_f();
      // initial recovery radius
      nav_rosette.recover_radius = nav_rosette.radius;
      nav_rosette.max_recover_radius = 2.0f * nav_rosette.recover_radius; // FIXME ?
      // init stage
      nav_init_stage();
      // recover
      nav_rosette.status = RSTT_RECOVER_OUTSIDE;
      break;
    case RSTT_RECOVER_OUTSIDE:
      // increment center position
      VECT3_ADD(nav_rosette.recover_circle, nav_rosette.pos_incr);
      nav_circle_XY(nav_rosette.recover_circle.x, nav_rosette.recover_circle.y , nav_rosette.radius_sign * nav_rosette.recover_radius);
      // increment recover circle radius
      if (nav_rosette.recover_radius < nav_rosette.max_recover_radius) {
        nav_rosette.recover_radius += 0.5;
      }
      // found a new border
      if (nav_rosette.inside_cloud) {
        nav_rosette.status = RSTT_CROSSING_START;
      }
      break;
    default:
      // error, leaving
      return false;
  }
  // update target and border positions
  VECT3_ADD(nav_rosette.estim_border, nav_rosette.pos_incr);
  VECT3_ADD(nav_rosette.target, nav_rosette.pos_incr);
#ifdef WP_TARGET
  nav_move_waypoint_enu(WP_TARGET, nav_rosette.target.x, nav_rosette.target.y, nav_rosette.target.z + ground_alt);
  RunOnceEvery(10, nav_send_waypoint(WP_TARGET));
#endif

  return true;
}
