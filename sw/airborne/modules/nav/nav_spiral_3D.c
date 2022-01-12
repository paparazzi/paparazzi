/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/nav/nav_spiral_3D.c
 *
 * Fixedwing navigation in a 3D spiral.
 *
 */

#include "modules/nav/nav_spiral_3D.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifndef NAV_SPIRAL_3D_DIST_DIFF
#define NAV_SPIRAL_3D_DIST_DIFF 10.f // horizontal distance difference before starting
#endif

#ifndef NAV_SPIRAL_3D_ALT_DIFF
#define NAV_SPIRAL_3D_ALT_DIFF 10.f // vertical distance difference before starting
#endif

#ifndef NAV_SPIRAL_3D_MIN_CIRCLE_RADIUS
#define NAV_SPIRAL_3D_MIN_CIRCLE_RADIUS 50.f
#endif

enum Spiral3DStatus { Spiral3DStart, Spiral3DCircle, Spiral3DFail };

struct NavSpiral3D {
  struct FloatVect3 center;
  struct FloatVect3 pos_incr;
  float alt_start;
  float alt_stop;
  float radius;
  float radius_min;
  float radius_start;
  float radius_stop;
  float radius_increment;
  enum Spiral3DStatus status;
};

struct NavSpiral3D nav_spiral_3D;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_spiral_3D_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit && nb == 9) {
    float cx = params[0];
    float cy = params[1];
    float alt_start = params[2];
    float alt_stop = params[3];
    float r_start = params[4];
    float r_stop = params[5];
    float vx = params[6];
    float vy = params[7];
    float vz = params[8];
    nav_spiral_3D_setup(cx, cy, alt_start, alt_stop, r_start, r_stop, vx, vy, vz);
    return true;
  }
  else if (flag == MissionUpdate && nb == 4) {
    // update current position and horizontal speed
    float cx = params[0];
    float cy = params[1];
    float vx = params[2];
    float vy = params[3];
    nav_spiral_3D.center.x = cx;
    nav_spiral_3D.center.y = cy;
    nav_spiral_3D.pos_incr.x = vx * nav_dt;
    nav_spiral_3D.pos_incr.y = vy * nav_dt;
    return true;
  }
  else if (flag == MissionUpdate && nb == 2) {
    // update horizontal speed
    float vx = params[0];
    float vy = params[1];
    nav_spiral_3D.pos_incr.x = vx * nav_dt;
    nav_spiral_3D.pos_incr.y = vy * nav_dt;
    return true;
  }
  else if (flag == MissionUpdate && nb == 1) {
    // update vertical speed
    float vz = params[0];
    nav_spiral_3D.pos_incr.z = vz * nav_dt;
    return true;
  }
  else if (flag == MissionRun) {
    return nav_spiral_3D_run();
  }
  return false; // not a valid case
}
#endif

void nav_spiral_3D_init(void)
{
  // no speed by default
  FLOAT_VECT3_ZERO(nav_spiral_3D.pos_incr);

#if USE_MISSION
  mission_register(nav_spiral_3D_mission, "SPIR3");
#endif
}

void nav_spiral_3D_setup(float center_x, float center_y,
                         float alt_start, float alt_stop,
                         float radius_start, float radius_stop,
                         float vx, float vy, float vz)
{
  // initial center position
  VECT3_ASSIGN(nav_spiral_3D.center, center_x, center_y, alt_start);
  nav_spiral_3D.alt_start = alt_start;
  nav_spiral_3D.alt_stop = alt_stop;
  float deltaZ = alt_stop - alt_start;
  if (deltaZ * vz < 0.f &&
      fabsf(deltaZ) > 1.f &&
      fabsf(radius_start - radius_stop) > 0.1f) {
    // error vz and delta Z don't have the same sign
    // and we are not doing a circle at constant alt
    nav_spiral_3D.status = Spiral3DFail;
    return;
  }
  // increment based on speed
  VECT3_ASSIGN(nav_spiral_3D.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);
  // radius
  nav_spiral_3D.radius_min = NAV_SPIRAL_3D_MIN_CIRCLE_RADIUS;
  nav_spiral_3D.radius_start = radius_start;
  if (nav_spiral_3D.radius_start < nav_spiral_3D.radius_min) {
    nav_spiral_3D.radius_start = nav_spiral_3D.radius_min;
  }
  nav_spiral_3D.radius = nav_spiral_3D.radius_start; // start radius
  nav_spiral_3D.radius_stop = radius_stop;
  if (nav_spiral_3D.radius_stop < nav_spiral_3D.radius_min) {
    nav_spiral_3D.radius_stop = nav_spiral_3D.radius_min;
  }
  // Compute radius increment
  float deltaR = radius_stop - radius_start;
  if (fabsf(deltaR) < 1.f) {
    // radius diff is too small we are doing a circle of constant radius
    nav_spiral_3D.radius_increment = 0.f;
  } else if (fabsf(deltaZ) < 1.f && fabsf(vz) > 0.1f) {
    // alt diff is too small, use Vz as increment rate at fix altitude
    // Rinc = deltaR / deltaT
    // deltaT = deltaR / Vz
    // Rinc = Vz
    float sign = deltaR < 0.f ? -1.f : 1.0;
    nav_spiral_3D.pos_incr.z = 0.f;
    nav_spiral_3D.radius_increment = sign * fabsf(vz) * nav_dt;
  } else if (fabsf(vz) < 0.1f) {
    // vz is too small, fail
    nav_spiral_3D.status = Spiral3DFail;
    return;
  } else {
    // normal case, vz and alt diff are large enough and in the same direction
    // Rinc = deltaR / deltaT
    // deltaT = deltaZ / Vz
    // Rinc = deltaR * Vz / deltaZ;
    nav_spiral_3D.radius_increment = deltaR * vz * nav_dt / deltaZ;
  }
  nav_spiral_3D.status = Spiral3DStart;
}

bool nav_spiral_3D_run(void)
{
  struct EnuCoor_f pos_enu = *stateGetPositionEnu_f();

  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, pos_enu, nav_spiral_3D.center);
  float dist_to_center = float_vect2_norm(&pos_diff);
  float dist_diff, alt_diff;
  float pre_climb = 0.f;

  switch (nav_spiral_3D.status) {
    case Spiral3DFail:
      // error during setup
      return false;

    case Spiral3DStart:
      // fly to start circle until dist to center and alt are acceptable
      // flys until center of the helix is reached and start helix
      VECT2_DIFF(pos_diff, pos_enu, nav_spiral_3D.center);
      dist_diff = fabs(dist_to_center - nav_spiral_3D.radius_start);
      alt_diff = fabs(stateGetPositionUtm_f()->alt - nav_spiral_3D.alt_start);
      if (dist_diff < NAV_SPIRAL_3D_DIST_DIFF && alt_diff < NAV_SPIRAL_3D_ALT_DIFF) {
        nav_spiral_3D.status = Spiral3DCircle;
      }
      break;

    case Spiral3DCircle:
      // increment center position
      VECT3_ADD(nav_spiral_3D.center, nav_spiral_3D.pos_incr);
      if (fabsf(nav_spiral_3D.radius_increment) > 0.001f) {
        // increment radius
        nav_spiral_3D.radius += nav_spiral_3D.radius_increment;
        // test end condition
        dist_diff = fabs(dist_to_center - nav_spiral_3D.radius_stop);
        alt_diff = fabs(stateGetPositionUtm_f()->alt - nav_spiral_3D.alt_stop);
        if (dist_diff < NAV_SPIRAL_3D_DIST_DIFF && alt_diff < NAV_SPIRAL_3D_ALT_DIFF) {
          // reaching desired altitude and radius
          return false; // spiral is finished
        }
      } else {
        // we are doing a circle of constant radius
        if (fabsf(nav_spiral_3D.pos_incr.z) > 0.1f) {
          // alt should change, check for arrival
          alt_diff = fabs(stateGetPositionUtm_f()->alt - nav_spiral_3D.alt_stop);
          if (alt_diff < NAV_SPIRAL_3D_ALT_DIFF) {
            // reaching desired altitude
            return false; // spiral is finished
          }
        }
      }
      pre_climb = nav_spiral_3D.pos_incr.z / nav_dt;
      break;

    default:
      break;
  }

  // horizontal control setting
  nav_circle_XY(nav_spiral_3D.center.x, nav_spiral_3D.center.y, nav_spiral_3D.radius);
  // vertical control setting
  NavVerticalAutoThrottleMode(0.); /* No pitch */
  NavVerticalAltitudeMode(nav_spiral_3D.center.z, pre_climb); /* No preclimb */

  return true;
}
