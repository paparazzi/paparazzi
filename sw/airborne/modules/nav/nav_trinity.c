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
 * @file "modules/nav/nav_trinity.c"
 * @author Titouan Verdu
 * Adaptative trinity pattern for cloud exploration.
 */

#include "modules/nav/nav_trinity.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "modules/core/abi.h"

#ifndef NAV_TRINITY_RECOVER_MAX_TURN
#define NAV_TRINITY_RECOVER_MAX_TURN 1.5f
#endif

#ifndef NAV_TRINITY_BORDER_FILTER
#define NAV_TRINITY_BORDER_FILTER 20.f
#endif

enum TrinityStatus {
  TRINITY_ENTER,
  TRINITY_INSIDE_FIRST,
  TRINITY_INSIDE,
  TRINITY_OUTSIDE_START,
  TRINITY_OUTSIDE,
  TRINITY_RECOVER_START,
  TRINITY_RECOVER_INSIDE,
  TRINITY_RECOVER_OUTSIDE
};

enum RotationDir {
  TRINITY_LEFT,
  TRINITY_RIGHT
};

struct NavTrinity {
  enum TrinityStatus status;
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

static struct NavTrinity nav_trinity;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static struct EnuCoor_f process_new_point_trinity(struct EnuCoor_f *position, float alt_sp, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_trinity.rotation == TRINITY_RIGHT) {
    rot_angle = -M_PI_2;
  } else{
    rot_angle = M_PI_2;
  }

  new_point.x = position->x + (cosf(rot_angle + uav_direction) * nav_trinity.radius);
  new_point.y = position->y + (sinf(rot_angle + uav_direction) * nav_trinity.radius);
  new_point.z = alt_sp;

  return new_point;
}

static struct EnuCoor_f process_first_point_trinity(struct EnuCoor_f *position, float alt_sp, float uav_direction)
{
  if (nav_trinity.rotation == TRINITY_RIGHT) {
    nav_trinity.rotation = TRINITY_LEFT;
  } else{
    nav_trinity.rotation = TRINITY_RIGHT;
  }

  return process_new_point_trinity(position, alt_sp, uav_direction);
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

static bool nav_trinity_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
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
    nav_trinity_setup(start_x, start_y, start_z, first_turn, circle_radius, vx, vy, vz);
    return true;
  }
  else if (flag == MissionUpdate && nb == 3) {
    // update target 3D position (ENU frame, above ground alt)
    float x = params[0];
    float y = params[1];
    float z = params[2];
    VECT3_ASSIGN(nav_trinity.target, x, y, z);
    return true;
  }
  else if (flag == MissionUpdate && nb == 2) {
    // update horizontal speed
    float vx = params[0];
    float vy = params[1];
    nav_trinity.pos_incr.x = vx * nav_dt;
    nav_trinity.pos_incr.y = vy * nav_dt;
    return true;
  }
  else if (flag == MissionUpdate && nb == 1) {
    // update vertical speed
    float vz = params[0];
    nav_trinity.pos_incr.z = vz * nav_dt;
    return true;
  }
  else if (flag == MissionRun) {
    return nav_trinity_run();
  }
  return false; // not a valid case
}
#endif

// ABI message

#ifndef NAV_TRINITY_LWC_ID
#define NAV_TRINITY_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static void lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data) {
  if (data_type == 1 && size == 1) {
    nav_trinity.inside_cloud = (bool) data[0];
  }
}

void nav_trinity_init(void)
{
  nav_trinity.status = TRINITY_ENTER;
  nav_trinity.radius = DEFAULT_CIRCLE_RADIUS;
  nav_trinity.recover_radius = DEFAULT_CIRCLE_RADIUS;
  nav_trinity.max_recover_radius = DEFAULT_CIRCLE_RADIUS;
  nav_trinity.last_border_time = 0.f;
  nav_trinity.inside_cloud = false;

  AbiBindMsgPAYLOAD_DATA(NAV_TRINITY_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_trinity_mission, "TRNTY");
#endif
}

void nav_trinity_setup(float init_x, float init_y,
                    float init_z, int turn,
                    float desired_radius, float vx,
                    float vy, float vz)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  // increment based on speed
  VECT3_ASSIGN(nav_trinity.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);

  nav_trinity.target = start;
  nav_trinity.status = TRINITY_ENTER;
  nav_trinity.inside_cloud = false;
  nav_trinity.radius = desired_radius;

  if (turn == 1) {
    nav_trinity.rotation = TRINITY_RIGHT;
    nav_trinity.radius_sign = 1.0f;
  } else {
    nav_trinity.rotation = TRINITY_LEFT;
    nav_trinity.radius_sign = -1.0f;
  }

  nav_trinity.actual = *stateGetPositionEnu_f();
}

bool nav_trinity_run(void)
{
  float pre_climb = 0.f;
  float time = get_sys_time_float();

  NavVerticalAutoThrottleMode(0.f); /* No pitch */

  switch (nav_trinity.status) {
    case TRINITY_ENTER:
      // init stage
      nav_init_stage();
      // reach target point
      nav_route_xy(nav_trinity.actual.x, nav_trinity.actual.y, nav_trinity.target.x, nav_trinity.target.y);
      NavVerticalAltitudeMode(nav_trinity.target.z + ground_alt, pre_climb);

      if (nav_trinity.inside_cloud) {
        nav_trinity.status = TRINITY_INSIDE_FIRST;
      }
      break;
    case TRINITY_INSIDE_FIRST:
      // prepare inside circle
      nav_trinity.actual = *stateGetPositionEnu_f();
      nav_trinity.direction = change_rep(stateGetHorizontalSpeedDir_f());
      nav_trinity.circle = process_first_point_trinity(&nav_trinity.actual, nav_trinity.target.z, nav_trinity.direction);
      // init stage
      nav_init_stage();
      // update border and target for recover
      nav_trinity.estim_border = nav_trinity.actual;
      update_target_point(&nav_trinity.target, &nav_trinity.estim_border, time - nav_trinity.last_border_time, NAV_TRINITY_BORDER_FILTER);
      nav_trinity.last_border_time = time;
      // fly inside
      nav_trinity.status = TRINITY_INSIDE;
      break;
    case TRINITY_INSIDE:
      // increment center position
      VECT3_ADD(nav_trinity.circle, nav_trinity.pos_incr);
      nav_circle_XY(nav_trinity.circle.x, nav_trinity.circle.y , nav_trinity.radius_sign * nav_trinity.radius);
      pre_climb = nav_trinity.pos_incr.z / nav_dt;
      NavVerticalAltitudeMode(nav_trinity.circle.z + ground_alt, pre_climb);

      if (!nav_trinity.inside_cloud) {
        // found border, start outside
        nav_trinity.status = TRINITY_OUTSIDE_START;
      }
      else if (NavCircleCountNoRewind() > NAV_TRINITY_RECOVER_MAX_TURN) {
        // most likely lost inside
        nav_trinity.status = TRINITY_RECOVER_START;
      }
      break;
    case TRINITY_OUTSIDE_START:
      // prepare outside circle
      nav_trinity.actual = *stateGetPositionEnu_f();
      nav_trinity.direction = change_rep(stateGetHorizontalSpeedDir_f());
      nav_trinity.circle = process_new_point_trinity(&nav_trinity.actual, nav_trinity.target.z, nav_trinity.direction);
      // init stage
      nav_init_stage();
      // upadte border and target for recover
      nav_trinity.estim_border = nav_trinity.actual;
      update_target_point(&nav_trinity.target, &nav_trinity.estim_border, time - nav_trinity.last_border_time, NAV_TRINITY_BORDER_FILTER);
      nav_trinity.last_border_time = time;
      // fly outside
      nav_trinity.status = TRINITY_OUTSIDE;
      break;
    case TRINITY_OUTSIDE:
      // increment center position
      VECT3_ADD(nav_trinity.circle, nav_trinity.pos_incr);
      pre_climb = nav_trinity.pos_incr.z / nav_dt;
      nav_circle_XY(nav_trinity.circle.x, nav_trinity.circle.y , nav_trinity.radius_sign * nav_trinity.radius);
      NavVerticalAltitudeMode(nav_trinity.circle.z + ground_alt, pre_climb);

      if(nav_trinity.inside_cloud){
        // found border, continue inside
        nav_trinity.status = TRINITY_INSIDE;
      }
      else if (NavCircleCountNoRewind() > NAV_TRINITY_RECOVER_MAX_TURN) {
        // most likely lost outside
        nav_trinity.status = TRINITY_RECOVER_START;
      }
      break;
    case TRINITY_RECOVER_START:
      // prepare recovery circle or line
      nav_trinity.recover_circle = nav_trinity.target;
      nav_trinity.actual = *stateGetPositionEnu_f();
      // initial recovery radius
      nav_trinity.recover_radius = nav_trinity.radius;
      nav_trinity.max_recover_radius = 2.0f * nav_trinity.recover_radius; // FIXME ?
      // init stage
      nav_init_stage();
      if (nav_trinity.inside_cloud) {
        nav_trinity.status = TRINITY_RECOVER_INSIDE;
      }
      else {
        nav_trinity.status = TRINITY_RECOVER_OUTSIDE;
      }
      break;
    case TRINITY_RECOVER_INSIDE:
      // increment border position
      // fly in opposite direction to cover more space
      nav_route_xy(nav_trinity.estim_border.x, nav_trinity.estim_border.y, nav_trinity.actual.x, nav_trinity.actual.y);
      if (!nav_trinity.inside_cloud) {
        nav_trinity.status = TRINITY_OUTSIDE_START;
      }
      break;
    case TRINITY_RECOVER_OUTSIDE:
      // increment center position
      VECT3_ADD(nav_trinity.recover_circle, nav_trinity.pos_incr);
      nav_circle_XY(nav_trinity.recover_circle.x, nav_trinity.recover_circle.y , nav_trinity.radius_sign * nav_trinity.recover_radius);
      // increment recover circle radius
      if (nav_trinity.recover_radius < nav_trinity.max_recover_radius) {
        nav_trinity.recover_radius += 0.5;
      }
      // found a new border
      if (nav_trinity.inside_cloud) {
        nav_trinity.status = TRINITY_INSIDE;
      }
      break;
    default:
      // error, leaving
      return false;
  }
  // increment border and target positions
  VECT3_ADD(nav_trinity.estim_border, nav_trinity.pos_incr);
  VECT3_ADD(nav_trinity.target, nav_trinity.pos_incr);
#ifdef WP_TARGET
  nav_move_waypoint_enu(WP_TARGET, nav_trinity.target.x, nav_trinity.target.y, nav_trinity.target.z + ground_alt);
  RunOnceEvery(10, nav_send_waypoint(WP_TARGET));
#endif

  return true;
}
