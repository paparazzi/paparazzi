/*
 *
 * Copyright (C) 2016, Michal Podhradsky, Thomas Fisher
 *
 * AggieAir, Utah State University
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
 *
 */

/**
 * @file modules/nav/nav_skid_landing.c
 * @brief Landing on skidpads
 * See video of the landing: https://www.youtube.com/watch?v=aYrB7s3oeX4
 * Standard landing procedure:
 * 1) circle down passing AF waypoint (from left or right)
 * 2) once low enough follow line to TD waypoint
 * 3) once low enough flare
 *
 * Use this in your airfame config file:
 *   <section name="LANDING" prefix="SKID_LANDING_">
 *     <define name="AF_HEIGHT" value="50" unit="m"/>
 *     <define name="FINAL_HEIGHT" value="50" unit="m"/>
 *     <define name="FINAL_STAGE_TIME" value="10" unit="s"/>
 *   </section>
 *
 *   Also define:
 *    V_CTL_LANDING_THROTTLE_PGAIN - landing throttle P gain
 *    V_CTL_LANDING_THROTTLE_IGAIN - landing throttle I gain
 *    V_CTL_LANDING_THROTTLE_MAX - max landing throttle
 *    V_CTL_LANDING_DESIRED_SPEED - desired landing speed
 *    V_CTL_LANDING_PITCH_PGAIN - landing P gain
 *    V_CTL_LANDING_PITCH_IGAIN - landing I gain
 *    V_CTL_LANDING_PITCH_LIMITS - pitch limits during landing
 *    V_CTL_LANDING_PITCH_FLARE - flare P gain
 *    V_CTL_LANDING_ALT_THROTTLE_KILL - AGL to kill throttle during landing
 *    V_CTL_LANDING_ALT_FLARE - AGL to initiate final flare
 *
 *  to properly use landing control loop
 */

#include "generated/airframe.h"
#include "state.h"
#include "modules/nav/nav_skid_landing.h"
#include "autopilot.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

enum LandingStatus
{
  CircleDown, LandingWait, Final, Approach
};
static enum LandingStatus skid_landing_status;
static uint8_t aw_waypoint;
static uint8_t td_waypoint;
static float land_radius;
static struct FloatVect2 land_circle;
static float land_app_alt;
static float land_circle_quadrant;
static float approach_quadrant;
static float final_land_altitude;
static uint8_t final_land_count;

#ifndef SKID_LANDING_AF_HEIGHT //> AF height [m]
#define SKID_LANDING_AF_HEIGHT 50
#endif
#ifndef SKID_LANDING_FINAL_HEIGHT //> final height [m]
#define SKID_LANDING_FINAL_HEIGHT 5
#endif
#ifndef SKID_LANDING_FINAL_STAGE_TIME //> final stage time [s]
#define SKID_LANDING_FINAL_STAGE_TIME 5
#endif

static inline float distance_equation(struct FloatVect2 p1,struct FloatVect2 p2)
{
  return sqrtf((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

void nav_skid_landing_setup(uint8_t afwp, uint8_t tdwp, float radius)
{
  aw_waypoint = afwp;
  td_waypoint = tdwp;
  skid_landing_status = CircleDown;
  land_radius = radius;
  land_app_alt = stateGetPositionUtm_f()->alt;
  final_land_altitude = SKID_LANDING_FINAL_HEIGHT;
  final_land_count = 1;

  float x_0 = waypoints[td_waypoint].x - waypoints[aw_waypoint].x;
  float y_0 = waypoints[td_waypoint].y - waypoints[aw_waypoint].y;

  /* Unit vector from AF to TD */
  float d = sqrtf(x_0 * x_0 + y_0 * y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  land_circle.x = waypoints[aw_waypoint].x + y_1 * land_radius;
  land_circle.y = waypoints[aw_waypoint].y - x_1 * land_radius;

  land_circle_quadrant = atan2(waypoints[aw_waypoint].x - land_circle.x,
      waypoints[aw_waypoint].y - land_circle.y);

  if (land_radius > 0) {
    approach_quadrant = land_circle_quadrant - RadOfDeg(90);
    land_circle_quadrant = land_circle_quadrant - RadOfDeg(45);
  } else {
    approach_quadrant = land_circle_quadrant + RadOfDeg(90);
    land_circle_quadrant = land_circle_quadrant + RadOfDeg(45);
  }
}

bool nav_skid_landing_run(void)
{
  switch (skid_landing_status) {
    case CircleDown:
      NavVerticalAutoThrottleMode(0);  // No pitch
      NavVerticalAltitudeMode(waypoints[aw_waypoint].a, 0);
      nav_circle_XY(land_circle.x, land_circle.y, land_radius);

      if (stateGetPositionUtm_f()->alt < waypoints[aw_waypoint].a + 5) {
        skid_landing_status = LandingWait;
        nav_init_stage();
      }

      break;

    case LandingWait:
      NavVerticalAutoThrottleMode(0);  // No pitch
      NavVerticalAltitudeMode(waypoints[aw_waypoint].a, 0);
      nav_circle_XY(land_circle.x, land_circle.y, land_radius);

      if (NavCircleCount() > 0.5 && NavQdrCloseTo(DegOfRad(approach_quadrant))) {
        skid_landing_status = Approach;
        nav_init_stage();
      }
      break;

    case Approach:
      NavVerticalAutoThrottleMode(0);  // No pitch
      NavVerticalAltitudeMode(waypoints[aw_waypoint].a, 0);
      nav_circle_XY(land_circle.x, land_circle.y, land_radius);

      if (NavQdrCloseTo(DegOfRad(land_circle_quadrant))) {
        skid_landing_status = Final;
        nav_init_stage();
      }
      break;

    case Final:
      if ((stateGetPositionUtm_f()->alt - waypoints[td_waypoint].a)
          < v_ctl_landing_alt_throttle_kill) {
        autopilot_set_kill_throttle(true);
      }
      nav_skid_landing_glide(aw_waypoint, td_waypoint);
      if (!autopilot_throttle_killed()) {
        nav_route_xy(waypoints[aw_waypoint].x, waypoints[aw_waypoint].y,
            waypoints[td_waypoint].x, waypoints[td_waypoint].y);
      }
      break;

    default:
      break;
  }
  return TRUE;
}

void nav_skid_landing_glide(uint8_t From_WP, uint8_t To_WP)
{
  struct FloatVect2 p_from;
  struct FloatVect2 p_to;
  struct FloatVect2 now;

  float start_alt = waypoints[From_WP].a;
  float diff_alt = waypoints[To_WP].a - start_alt;
  p_from.x = waypoints[From_WP].x;
  p_from.y = waypoints[From_WP].y;
  p_to.x = waypoints[To_WP].x;
  p_to.y = waypoints[To_WP].y;
  now.x = stateGetPositionEnu_f()->x;
  now.y = stateGetPositionEnu_f()->y;

  float end_dist = distance_equation(p_from,p_to);
  float here_dist = distance_equation(p_from,now);
  float end_here_dist = distance_equation(p_to,now);
  float alt = start_alt + (here_dist/end_dist) * diff_alt;

  if (end_dist < end_here_dist){
  alt = start_alt;
  }
   v_ctl_mode = V_CTL_MODE_LANDING;

  if ((stateGetPositionUtm_f()->alt-waypoints[To_WP].a)<0.0){
    alt = waypoints[To_WP].a;
   }
   nav_altitude = alt;
}
