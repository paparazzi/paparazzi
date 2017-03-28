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
 * @file modules/nav/nav_launcher.c
 * @brief Pneumatic launcher system
 * See video of the system: https://www.youtube.com/watch?v=qc1uwH-8Dbw
 *       Launcher.
 *    A pneumatic launching system.
 *    - Phase 1: Zero Roll, Takeoff Pitch, Full Throttle(once you enter the block!)
 *    - Phase 2: After detecting lauch (ground speed) and travelling enough distance from the launch point
 *               follow launch line -> Auto roll, Takeoff pitch,  Full Throttle
 *    - Phase 3: If the aircraft is above a specific alt, greater than a specific speed or too far away, circle up
 *               with takeoff circle radius, until you reach desired takeoff altitude
 *
 *   An example section to be added into your airframe configuration:
 *   <!-- Launcher Takeoff Configuration -->
 *   <section name="LAUNCHER" prefix="LAUNCHER_TAKEOFF_">
 *     <define name="PITCH" value="0.23" unit="rad"/>
 *     <define name="HEIGH" value="70" unit="m"/>
 *     <define name="MIN_SPEED_CIRCLE" value="8" unit="m/s"/>
 *     <define name="DISTANCE" value="30" unit="m"/>
 *     <define name="MIN_SPEED_LINE" value="5" unit="m/s"/>
 *   </section>
 */

#include "generated/airframe.h"
#include "state.h"
#include "modules/nav/nav_launcher.h"
#include "autopilot.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

#ifndef LAUNCHER_TAKEOFF_PITCH //> desired takeoff pitch [rad]
#define LAUNCHER_TAKEOFF_PITCH 0.23
#endif

#ifndef LAUNCHER_TAKEOFF_HEIGHT //> desired height AGL  [m]
#define LAUNCHER_TAKEOFF_HEIGHT 80
#endif

#ifndef LAUNCHER_TAKEOFF_DISTANCE //> minimal distance from the takeoff point [m]
#define LAUNCHER_TAKEOFF_DISTANCE 30
#endif

#ifndef LAUNCHER_TAKEOFF_MIN_SPEED_LINE //> minimal takeoff ground speed [m/s] to switch into line following
#define LAUNCHER_TAKEOFF_MIN_SPEED_LINE 5
#endif

#ifndef LAUNCHER_TAKEOFF_MIN_SPEED_CIRCLE //> mininmal takeoff ground speed [m/s] to switch into circle climb
#define LAUNCHER_TAKEOFF_MIN_SPEED_CIRCLE 8
#endif

#ifndef LAUNCHER_TAKEOFF_CIRCLE_ALT //> desired circle AGL [m]
#define LAUNCHER_TAKEOFF_CIRCLE_ALT 200
#endif

#ifndef LAUNCHER_TAKEOFF_CIRCLE_RADIUS //> desired circle radius [m]
#define LAUNCHER_TAKEOFF_CIRCLE_RADIUS 200
#endif

#ifndef LAUNCHER_TAKEOFF_MAX_CIRCLE_DISTANCE //> max distance from the takeoff point [m] before the plane circles up
#define LAUNCHER_TAKEOFF_MAX_CIRCLE_DISTANCE 800
#endif

#ifndef LAUNCHER_TAKEOFF_HEIGHT_THRESHOLD //> height threshold [m] before switching modes
#define LAUNCHER_TAKEOFF_HEIGHT_THRESHOLD 10
#endif

struct Point2D
{
  float x;
  float y;
};
enum Launch_Status
{
  L_Pitch_Nav, L_Line_Nav, L_CircleUp, L_Finished
};
static enum Launch_Status CLaunch_Status;
static float launch_x;
static float launch_y;
static float launch_alt;
static float launch_pitch;
static float launch_time;
static struct Point2D launch_circle;
static float launch_circle_alt;

static float launch_line_x;
static float launch_line_y;

void nav_launcher_setup(void)
{
  launch_x = stateGetPositionEnu_f()->x;
  launch_y = stateGetPositionEnu_f()->y;
  launch_alt = stateGetPositionUtm_f()->alt + LAUNCHER_TAKEOFF_HEIGHT;
  launch_pitch = stateGetNedToBodyEulers_f()->theta;
  launch_time = 0;

  launch_circle_alt =
      stateGetPositionUtm_f()->alt + LAUNCHER_TAKEOFF_CIRCLE_ALT;

  CLaunch_Status = L_Pitch_Nav;
}

bool nav_launcher_run(void)
{
  //Find distance from laucher
  float dist_x = stateGetPositionEnu_f()->x - launch_x;
  float dist_y = stateGetPositionEnu_f()->y - launch_y;
  float launch_dist = sqrtf(dist_x * dist_x + dist_y * dist_y);
  if (launch_dist <= 0.01) {
    launch_dist = 0.01;
  }

  switch (CLaunch_Status) {
    case L_Pitch_Nav:
      //Follow Launch Line
      NavVerticalAltitudeMode(launch_alt, 0);
      NavVerticalAutoThrottleMode(LAUNCHER_TAKEOFF_PITCH);
      NavVerticalThrottleMode(MAX_PPRZ * (1));
      NavAttitude(0);


      //If the plane has been launched and has traveled for more than a specified distance, switch to line nav
      if (stateGetHorizontalSpeedNorm_f() > LAUNCHER_TAKEOFF_MIN_SPEED_LINE) {
        if (launch_dist > LAUNCHER_TAKEOFF_DISTANCE) {
          launch_line_x = stateGetPositionEnu_f()->x;
          launch_line_y = stateGetPositionEnu_f()->y;
          CLaunch_Status = L_Line_Nav;
        }
      }

      break;
    case L_Line_Nav:
      //Follow Launch Line
      NavVerticalAltitudeMode(launch_alt, 0);
      NavVerticalAutoThrottleMode(LAUNCHER_TAKEOFF_PITCH);
      NavVerticalThrottleMode(MAX_PPRZ * (1));
      nav_route_xy(launch_x, launch_y, launch_line_x, launch_line_y);

      //If the aircraft is above a specific alt, greater than a specific speed or too far away, circle up
      if (((stateGetPositionUtm_f()->alt
          > (launch_alt - LAUNCHER_TAKEOFF_HEIGHT_THRESHOLD))
          && (stateGetHorizontalSpeedNorm_f()
              > LAUNCHER_TAKEOFF_MIN_SPEED_CIRCLE))
          || (launch_dist > LAUNCHER_TAKEOFF_MAX_CIRCLE_DISTANCE)) {
        CLaunch_Status = L_CircleUp;

        //Find position of circle
        float x_1 = dist_x / launch_dist;
        float y_1 = dist_y / launch_dist;

        launch_circle.x = stateGetPositionEnu_f()->x
            + y_1 * LAUNCHER_TAKEOFF_CIRCLE_RADIUS;
        launch_circle.y = stateGetPositionEnu_f()->y
            - x_1 * LAUNCHER_TAKEOFF_CIRCLE_RADIUS;
      }
      break;
    case L_CircleUp:
      NavVerticalAutoThrottleMode(0);
      NavVerticalAltitudeMode(launch_circle_alt, 0);
      nav_circle_XY(launch_circle.x, launch_circle.y,
          LAUNCHER_TAKEOFF_CIRCLE_RADIUS);

      if (stateGetPositionUtm_f()->alt
          > (launch_circle_alt - LAUNCHER_TAKEOFF_HEIGHT_THRESHOLD)) {
        CLaunch_Status = L_Finished;
        return FALSE;
      }
      break;
    default:
      break;
  }
  return TRUE;
}

