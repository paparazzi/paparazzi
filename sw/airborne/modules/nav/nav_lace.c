/*
 * Copyright (C) 2018-2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/nav/nav_lace.c
 * @author VERDU Titouan
 *
 * Adaptive border pattern for cloud exploration
 */

#include "modules/nav/nav_lace.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "modules/core/abi.h"

#ifndef NAV_LACE_RECOVER_MAX_TURN
#define NAV_LACE_RECOVER_MAX_TURN 1.5f
#endif

#ifndef NAV_LACE_BORDER_FILTER
#define NAV_LACE_BORDER_FILTER 20.f
#endif

enum LaceStatus {
  LACE_ENTER,
  LACE_INSIDE_START,
  LACE_INSIDE,
  LACE_OUTSIDE_START,
  LACE_OUTSIDE,
  LACE_RECOVER_START,
  LACE_RECOVER_INSIDE,
  LACE_RECOVER_OUTSIDE
};

enum RotationDir {
  LACE_LEFT,
  LACE_RIGHT
};

struct NavLace {
  enum LaceStatus status;
  enum RotationDir rotation;
  bool inside_cloud;
  struct EnuCoor_f actual;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  struct EnuCoor_f estim_border;
  struct EnuCoor_f recover_circle;
  struct FloatVect3 pos_incr;
  float direction;
  float radius;
  float radius_sign;
  float last_border_time;
  float recover_radius;
  float max_recover_radius;
};

static struct NavLace nav_lace;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static struct EnuCoor_f process_new_point_lace(struct EnuCoor_f *position, float alt_sp, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_lace.rotation == LACE_RIGHT) {
    rot_angle = -M_PI_2;
    nav_lace.rotation = LACE_LEFT;
  } else {
    rot_angle = M_PI_2;
    nav_lace.rotation = LACE_RIGHT;
  }

  new_point.x = position->x + (cosf(rot_angle + uav_direction) * nav_lace.radius);
  new_point.y = position->y + (sinf(rot_angle + uav_direction) * nav_lace.radius);
  new_point.z = alt_sp;

  return new_point;
}

static void update_target_point(struct EnuCoor_f *target, struct EnuCoor_f *border, float dt, float tau)
{
  // with a proper discrete transform, coeff should be exp(-dt/tau) and (1-exp(-dt/tau))
  // but to make it simpler with the same behavior at the limits, we use tau/(dt+tau) and dt/(dt+tau)
  // with a positive value for tau
  if (tau > 1e-4) {
    float alpha = dt / (dt + tau);
    target->x = (border->x * alpha) + (target->x * (1.f - alpha));
    target->y = (border->y * alpha) + (target->y * (1.f - alpha));
  }
}


#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_lace_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
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
    nav_lace_setup(start_x, start_y, start_z, first_turn, circle_radius, vx, vy, vz);
    return true;
  }
  else if (flag == MissionUpdate && nb == 3) {
    // update target 3D position (ENU frame, above ground alt)
    float x = params[0];
    float y = params[1];
    float z = params[2];
    VECT3_ASSIGN(nav_lace.target, x, y, z);
    return true;
  }
  else if (flag == MissionUpdate && nb == 2) {
    // update horizontal speed
    float vx = params[0];
    float vy = params[1];
    nav_lace.pos_incr.x = vx * nav_dt;
    nav_lace.pos_incr.y = vy * nav_dt;
    return true;
  }
  else if (flag == MissionUpdate && nb == 1) {
    // update vertical speed
    float vz = params[0];
    nav_lace.pos_incr.z = vz * nav_dt;
    return true;
  }
  else if (flag == MissionRun) {
    return nav_lace_run();
  }
  return false; // not a valid case
}
#endif

// ABI message

#ifndef NAV_LACE_LWC_ID
#define NAV_LACE_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static void lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data) {
  if (data_type == 1 && size == 1) {
    nav_lace.inside_cloud = (bool) data[0];
  }
}

void nav_lace_init(void)
{
  nav_lace.status = LACE_ENTER;
  nav_lace.radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.recover_radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.max_recover_radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.last_border_time = 0.f;
  nav_lace.inside_cloud = false;

  AbiBindMsgPAYLOAD_DATA(NAV_LACE_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_lace_mission, "LACE");
#endif
}

void nav_lace_setup(float init_x, float init_y,
                    float init_z, int turn,
                    float desired_radius, float vx,
                    float vy, float vz)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  // increment based on speed
  VECT3_ASSIGN(nav_lace.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);

  nav_lace.target = start;
  nav_lace.status = LACE_ENTER;
  nav_lace.inside_cloud = false;
  nav_lace.radius = desired_radius;

  if (turn == 1) {
    nav_lace.rotation = LACE_RIGHT;
    nav_lace.radius_sign = 1.0f;
  } else {
    nav_lace.rotation = LACE_LEFT;
    nav_lace.radius_sign = -1.0f;
  }

  nav_lace.actual = *stateGetPositionEnu_f();
}

bool nav_lace_run(void)
{
  float pre_climb = 0.f;
  float time = get_sys_time_float();

  NavVerticalAutoThrottleMode(0.f); /* No pitch */

  switch (nav_lace.status) {
    case LACE_ENTER:
      // init stage
      nav_init_stage();
      // reach target point
      nav_route_xy(nav_lace.actual.x, nav_lace.actual.y, nav_lace.target.x, nav_lace.target.y);
      NavVerticalAltitudeMode(nav_lace.target.z + ground_alt, pre_climb);

      if (nav_lace.inside_cloud) {
        // found border or already inside
        nav_lace.status = LACE_INSIDE_START;
        // update target for horizontal position
        nav_lace.target.x = stateGetPositionEnu_f()->x;
        nav_lace.target.y = stateGetPositionEnu_f()->y;
      }
      break;
    case LACE_INSIDE_START:
      // prepare inside circle
      nav_lace.actual = *stateGetPositionEnu_f();
      nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
      nav_lace.circle = process_new_point_lace(&nav_lace.actual, nav_lace.target.z, nav_lace.direction);
      // init stage
      nav_init_stage();
      // update border and target for recover
      nav_lace.estim_border = nav_lace.actual;
      update_target_point(&nav_lace.target, &nav_lace.estim_border, time - nav_lace.last_border_time, NAV_LACE_BORDER_FILTER);
      nav_lace.last_border_time = time;
      // fly inside
      nav_lace.status = LACE_INSIDE;
      break;
    case LACE_INSIDE:
      // increment center position
      VECT3_ADD(nav_lace.circle, nav_lace.pos_incr);
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      pre_climb = nav_lace.pos_incr.z / nav_dt;
      NavVerticalAltitudeMode(nav_lace.circle.z + ground_alt, pre_climb);

      if (!nav_lace.inside_cloud) {
        // found border, start outside
        nav_lace.status = LACE_OUTSIDE_START;
        nav_lace.radius_sign = -1.0f * nav_lace.radius_sign;
      }
      else if (NavCircleCountNoRewind() > NAV_LACE_RECOVER_MAX_TURN) {
        // most likely lost inside
        nav_lace.status = LACE_RECOVER_START;
      }
      break;
    case LACE_OUTSIDE_START:
      // prepare outside circle
      nav_lace.actual = *stateGetPositionEnu_f();
      nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
      nav_lace.circle = process_new_point_lace(&nav_lace.actual, nav_lace.circle.z, nav_lace.direction);
      // init stage
      nav_init_stage();
      // upadte border and target for recover
      nav_lace.estim_border = nav_lace.actual;
      update_target_point(&nav_lace.target, &nav_lace.estim_border, time - nav_lace.last_border_time, NAV_LACE_BORDER_FILTER);
      nav_lace.last_border_time = time;
      // fly outside
      nav_lace.status = LACE_OUTSIDE;
      break;
    case LACE_OUTSIDE:
      // increment center position
      VECT3_ADD(nav_lace.circle, nav_lace.pos_incr);
      pre_climb = nav_lace.pos_incr.z / nav_dt;
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      NavVerticalAltitudeMode(nav_lace.circle.z + ground_alt, pre_climb);

      if (nav_lace.inside_cloud) {
        // found border, start inside
        nav_lace.status = LACE_INSIDE_START;
        nav_lace.radius_sign = -1.0f * nav_lace.radius_sign;
      }
      else if (NavCircleCountNoRewind() > NAV_LACE_RECOVER_MAX_TURN) {
        // most likely lost outside
        nav_lace.status = LACE_RECOVER_START;
      }
      break;
    case LACE_RECOVER_START:
      // prepare recovery circle or line
      nav_lace.recover_circle = nav_lace.target;
      nav_lace.actual = *stateGetPositionEnu_f();
      // initial recovery radius
      nav_lace.recover_radius = nav_lace.radius;
      nav_lace.max_recover_radius = 2.0f * nav_lace.recover_radius; // FIXME ?
      // init stage
      nav_init_stage();
      if (nav_lace.inside_cloud) {
        nav_lace.status = LACE_RECOVER_INSIDE;
      }
      else {
        nav_lace.status = LACE_RECOVER_OUTSIDE;
      }
      break;
    case LACE_RECOVER_INSIDE:
      // increment border position
      // fly in opposite direction to cover more space
      nav_route_xy(nav_lace.estim_border.x, nav_lace.estim_border.y, nav_lace.actual.x, nav_lace.actual.y);
      if (!nav_lace.inside_cloud) {
        nav_lace.status = LACE_OUTSIDE_START;
        nav_lace.radius_sign = -1.0f * nav_lace.radius_sign;
      }
      break;
    case LACE_RECOVER_OUTSIDE:
      // increment center position
      VECT3_ADD(nav_lace.recover_circle, nav_lace.pos_incr);
      nav_circle_XY(nav_lace.recover_circle.x, nav_lace.recover_circle.y , nav_lace.radius_sign * nav_lace.recover_radius);
      // increment recover circle radius
      if (nav_lace.recover_radius < nav_lace.max_recover_radius) {
        nav_lace.recover_radius += 0.5;
      }
      // found a new border
      if (nav_lace.inside_cloud) {
        nav_lace.status = LACE_INSIDE_START;
        nav_lace.radius_sign = -1.0f * nav_lace.radius_sign;
      }
      break;
    default:
      // error, leaving
      return false;
  }
  // increment border and target positions
  VECT3_ADD(nav_lace.estim_border, nav_lace.pos_incr);
  VECT3_ADD(nav_lace.target, nav_lace.pos_incr);
#ifdef WP_TARGET
  nav_move_waypoint_enu(WP_TARGET, nav_lace.target.x, nav_lace.target.y, nav_lace.target.z + ground_alt);
  RunOnceEvery(10, nav_send_waypoint(WP_TARGET));
#endif

  return true;
}
