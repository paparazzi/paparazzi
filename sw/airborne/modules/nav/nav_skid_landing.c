/*
 *
 * Copyright (C) 2016, Michal Podhradsky
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
 * @file modules/nav/nav_skid_landiung.c
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
 */

#include "generated/airframe.h"
#include "state.h"
#include "modules/nav/nav_skid_landing.h"
#include "firmwares/fixedwing/autopilot.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

struct Point2D
{
  float x;
  float y;
};
enum LandingStatus
{
  CircleDown, LandingWait, Final, Approach
};
static enum LandingStatus CLandingStatus;
static uint8_t AFWaypoint;
static uint8_t TDWaypoint;
static float LandRadius;
static struct Point2D LandCircle;
static float LandAppAlt;
static float LandCircleQDR;
static float ApproachQDR;
static float FinalLandAltitude;
static uint8_t FinalLandCount;

#ifndef SKID_LANDING_AF_HEIGHT //> AF height [m]
#define SKID_LANDING_AF_HEIGHT 50
#endif
#ifndef SKID_LANDING_FINAL_HEIGHT //> final height [m]
#define SKID_LANDING_FINAL_HEIGHT 5
#endif
#ifndef SKID_LANDING_FINAL_STAGE_TIME //> final stage time [s]
#define SKID_LANDING_FINAL_STAGE_TIME 5
#endif

static inline float distance_equation(struct Point2D p1,struct Point2D p2)
{
  return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

bool nav_skid_landing_setup(uint8_t AFWP, uint8_t TDWP, float radius)
{
  AFWaypoint = AFWP;
  TDWaypoint = TDWP;
  CLandingStatus = CircleDown;
  LandRadius = radius;
  LandAppAlt = stateGetPositionUtm_f()->alt;
  FinalLandAltitude = SKID_LANDING_FINAL_HEIGHT;
  FinalLandCount = 1;

  float x_0 = waypoints[TDWaypoint].x - waypoints[AFWaypoint].x;
  float y_0 = waypoints[TDWaypoint].y - waypoints[AFWaypoint].y;

  /* Unit vector from AF to TD */
  float d = sqrt(x_0 * x_0 + y_0 * y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  LandCircle.x = waypoints[AFWaypoint].x + y_1 * LandRadius;
  LandCircle.y = waypoints[AFWaypoint].y - x_1 * LandRadius;

  LandCircleQDR = atan2(waypoints[AFWaypoint].x - LandCircle.x,
      waypoints[AFWaypoint].y - LandCircle.y);

  if (LandRadius > 0) {
    ApproachQDR = LandCircleQDR - RadOfDeg(90);
    LandCircleQDR = LandCircleQDR - RadOfDeg(45);
  } else {
    ApproachQDR = LandCircleQDR + RadOfDeg(90);
    LandCircleQDR = LandCircleQDR + RadOfDeg(45);
  }

  return FALSE;
}

bool nav_skid_landing_run(void)
{
  switch (CLandingStatus) {
    case CircleDown:
      NavVerticalAutoThrottleMode(0);  // No pitch
      NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
      nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

      if (stateGetPositionUtm_f()->alt < waypoints[AFWaypoint].a + 5) {
        CLandingStatus = LandingWait;
        nav_init_stage();
      }

      break;

    case LandingWait:
      NavVerticalAutoThrottleMode(0);  // No pitch
      NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
      nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

      if (NavCircleCount() > 0.5 && NavQdrCloseTo(DegOfRad(ApproachQDR))) {
        CLandingStatus = Approach;
        nav_init_stage();
      }
      break;

    case Approach:
      NavVerticalAutoThrottleMode(0);  // No pitch
      NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
      nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

      if (NavQdrCloseTo(DegOfRad(LandCircleQDR))) {
        CLandingStatus = Final;
        nav_init_stage();
      }
      break;

    case Final:
      if ((stateGetPositionUtm_f()->alt - waypoints[TDWaypoint].a)
          < v_ctl_landing_alt_throttle_kill) {
        kill_throttle = 1;
      }
      nav_skid_landing_glide(AFWaypoint, TDWaypoint);
      if (!kill_throttle) {
        nav_route_xy(waypoints[AFWaypoint].x, waypoints[AFWaypoint].y,
            waypoints[TDWaypoint].x, waypoints[TDWaypoint].y);
      }
      break;

    default:
      break;
  }
  return TRUE;
}

void nav_skid_landing_glide(uint8_t From_WP, uint8_t To_WP)
{
  struct Point2D p_from;
  struct Point2D p_to;
  struct Point2D now;

  float start_alt = waypoints[From_WP].a;
  float diff_alt = waypoints[To_WP].a - start_alt;
  p_from.x = waypoints[From_WP].x;
  p_from.y = waypoints[From_WP].y;
  p_to.x = waypoints[To_WP].x;
  p_to.y = waypoints[To_WP].y;
  now.x = stateGetPositionEnu_f()->x;
  now.y = stateGetPositionEnu_f()->y;

  float EndDist = distance_equation(p_from,p_to);
  float HereDist = distance_equation(p_from,now);
  float EndHereDist = distance_equation(p_to,now);
  float alt = start_alt + (HereDist/EndDist) * diff_alt;

  if (EndDist < EndHereDist){
  alt = start_alt;
  }
   v_ctl_mode = V_CTL_MODE_LANDING;

  if ((stateGetPositionUtm_f()->alt-waypoints[To_WP].a)<0.0){
    alt = waypoints[To_WP].a;
   }
   nav_altitude = alt;
}
