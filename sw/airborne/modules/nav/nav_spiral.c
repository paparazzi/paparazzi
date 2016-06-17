/*
 * Copyright (C) 2011-2013  The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/nav/nav_spiral.c
 *
 * Fixedwing navigation in a spiral/helix.
 *
 * creating a helix:
 * - start radius to end radius, increasing after reaching alphamax
 * - Alphamax is calculated from given segments
 * - IMPORTANT: numer of segments has to be larger than 2!
 */

#include "modules/nav/nav_spiral.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

#ifndef NAV_SPIRAL_MIN_CIRCLE_RADIUS
#define NAV_SPIRAL_MIN_CIRCLE_RADIUS 60
#endif

struct NavSpiral nav_spiral;

bool nav_spiral_setup(uint8_t center_wp, uint8_t edge_wp, float radius_start, float radius_inc, float segments)
{
  VECT2_COPY(nav_spiral.center, waypoints[center_wp]);    // center of the helix
  nav_spiral.center.z = waypoints[center_wp].a;
  nav_spiral.radius_start = radius_start;   // start radius of the helix
  nav_spiral.segments = segments;
  nav_spiral.radius_min = NAV_SPIRAL_MIN_CIRCLE_RADIUS;
  if (nav_spiral.radius_start < nav_spiral.radius_min) {
    nav_spiral.radius_start = nav_spiral.radius_min;
  }
  nav_spiral.radius_increment = radius_inc;     // multiplier for increasing the spiral

  struct FloatVect2 edge;
  VECT2_DIFF(edge, waypoints[edge_wp], nav_spiral.center);

  nav_spiral.radius = float_vect2_norm(&edge);

  // get a copy of the current position
  struct EnuCoor_f pos_enu = *stateGetPositionEnu_f();

  VECT2_DIFF(nav_spiral.trans_current, pos_enu, nav_spiral.center);
  nav_spiral.trans_current.z = stateGetPositionUtm_f()->alt - nav_spiral.center.z;

  nav_spiral.dist_from_center = FLOAT_VECT3_NORM(nav_spiral.trans_current);

  // nav_spiral.alpha_limit denotes angle, where the radius will be increased
  nav_spiral.alpha_limit = 2 * M_PI / nav_spiral.segments;
  //current position
  nav_spiral.fly_from.x = stateGetPositionEnu_f()->x;
  nav_spiral.fly_from.y = stateGetPositionEnu_f()->y;

  if (nav_spiral.dist_from_center > nav_spiral.radius) {
    nav_spiral.status = SpiralOutside;
  }
  return false;
}

bool nav_spiral_run(void)
{
  struct EnuCoor_f pos_enu = *stateGetPositionEnu_f();

  VECT2_DIFF(nav_spiral.trans_current, pos_enu, nav_spiral.center);
  nav_spiral.dist_from_center = FLOAT_VECT3_NORM(nav_spiral.trans_current);

  float DistanceStartEstim;
  float CircleAlpha;

  switch (nav_spiral.status) {
    case SpiralOutside:
      //flys until center of the helix is reached an start helix
      nav_route_xy(nav_spiral.fly_from.x, nav_spiral.fly_from.y, nav_spiral.center.x, nav_spiral.center.y);
      // center reached?
      if (nav_approaching_xy(nav_spiral.center.x, nav_spiral.center.y, nav_spiral.fly_from.x, nav_spiral.fly_from.y, 0)) {
        // nadir image
#ifdef DIGITAL_CAM
        dc_send_command(DC_SHOOT);
#endif
        nav_spiral.status = SpiralStartCircle;
      }
      break;
    case SpiralStartCircle:
      // Starts helix
      // storage of current coordinates
      // calculation needed, State switch to SpiralCircle
      nav_circle_XY(nav_spiral.center.y, nav_spiral.center.y, nav_spiral.radius_start);
      if (nav_spiral.dist_from_center >= nav_spiral.radius_start) {
        VECT2_COPY(nav_spiral.last_circle, pos_enu);
        nav_spiral.status = SpiralCircle;
        // Start helix
#ifdef DIGITAL_CAM
        dc_Circle(360 / nav_spiral.segments);
#endif
      }
      break;
    case SpiralCircle: {
      nav_circle_XY(nav_spiral.center.x, nav_spiral.center.y, nav_spiral.radius_start);
      // Trigonometrische Berechnung des bereits geflogenen Winkels alpha
      // equation:
      // alpha = 2 * asin ( |Starting position angular - current positon| / (2* nav_spiral.radius_start)
      // if alphamax already reached, increase radius.

      //DistanceStartEstim = |Starting position angular - current positon|
      struct FloatVect2 pos_diff;
      VECT2_DIFF(pos_diff, nav_spiral.last_circle, pos_enu);
      DistanceStartEstim = float_vect2_norm(&pos_diff);
      CircleAlpha = (2.0 * asin(DistanceStartEstim / (2 * nav_spiral.radius_start)));
      if (CircleAlpha >= nav_spiral.alpha_limit) {
        VECT2_COPY(nav_spiral.last_circle, pos_enu);
        nav_spiral.status = SpiralInc;
      }
      break;
    }
    case SpiralInc:
      // increasing circle radius as long as it is smaller than max helix radius
      if (nav_spiral.radius_start + nav_spiral.radius_increment < nav_spiral.radius) {
        nav_spiral.radius_start = nav_spiral.radius_start + nav_spiral.radius_increment;
#ifdef DIGITAL_CAM
        if (dc_cam_tracing) {
          // calculating Cam angle for camera alignment
          nav_spiral.trans_current.z = stateGetPositionUtm_f()->alt - nav_spiral.center.z;
          dc_cam_angle = atan(nav_spiral.radius_start / nav_spiral.trans_current.z) * 180  / M_PI;
        }
#endif
      } else {
        nav_spiral.radius_start = nav_spiral.radius;
#ifdef DIGITAL_CAM
        // Stopps DC
        dc_stop();
#endif
      }
      nav_spiral.status = SpiralCircle;
      break;
    default:
      break;
  }

  NavVerticalAutoThrottleMode(0.); /* No pitch */
  NavVerticalAltitudeMode(nav_spiral.center.z, 0.); /* No preclimb */

  return true;
}
