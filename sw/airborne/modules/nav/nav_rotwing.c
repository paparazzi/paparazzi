/*
 * Copyright (C) 2024 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/nav/nav_rotwing.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Custom mission nav patterns for the rotating wing drone
 */

#include "modules/nav/nav_rotwing.h"
#include "navigation.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "modules/rotwing_drone/rotwing_state.h"

#include "generated/flight_plan.h" // TODO. Make fp independent
#include "sonar/agl_dist.h"

// navigation time step
static const float dt_navigation = 1.0f / ((float)NAVIGATION_FREQUENCY);

enum TakeoffStatus {
  StartEngine,
  RunEngine,
  Takeoff_init,
  Takeoff,
  Climb_init,
  Climb,
  ErrorCase
};

enum LandingStatus {
  Descend,
  Flare,
  Flarelow
};

static enum TakeoffStatus rotwing_takeoff_status = StartEngine;
static enum LandingStatus rotwing_landing_status = Descend;

static float takeoff_timer = 0.0f;
static float landing_timer = 0.0f;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_rotwing_takeoff(uint8_t nb __attribute__((unused)), float *params __attribute__((unused)), enum MissionRunFlag flag)
{
  if (flag == MissionInit) {
    if (!autopilot.in_flight) {
      rotwing_takeoff_status = StartEngine;
      //printf("Start_engine\n");
    } else {
      rotwing_takeoff_status = Climb;
    }
    takeoff_timer = 0.0f;
    return nav_rotwing_takeoff_run();
  }
  else if (flag == MissionRun) {
    return nav_rotwing_takeoff_run();
  }
  return false;
}

static bool nav_rotwing_land(uint8_t nb __attribute__((unused)), float *params __attribute__((unused)), enum MissionRunFlag flag)
{
  if (flag == MissionInit) {
    rotwing_state_set(ROTWING_STATE_REQUEST_HOVER);
    rotwing_landing_status = Descend;
    landing_timer = 0.0f;
    return nav_rotwing_land_run();
  }
  else if (flag == MissionRun) {
    return nav_rotwing_land_run();
  }
  return false;
}

#endif // USE_MISSION

void nav_rotwing_init(void)
{
  #if USE_MISSION
    mission_register(nav_rotwing_takeoff, "TO");
    mission_register(nav_rotwing_land, "LAND");
  #endif
}

bool nav_rotwing_takeoff_run(void)
{
  switch (rotwing_takeoff_status) {
    case StartEngine:
      rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
      NavResurrect();
      nav_set_heading_current();
      NavAttitude((stateGetNedToBodyEulers_f())->phi);
      NavVerticalAutoThrottleMode((stateGetNedToBodyEulers_f())->theta);
      NavVerticalThrottleMode(9600*(0));
      // Switch to next state
      if (rotwing_state_hover_motors_running()) {
        takeoff_timer = 0.0f;
        rotwing_takeoff_status = RunEngine;
      }
      // Timeout
      if (takeoff_timer > 10.0) {
        rotwing_takeoff_status = ErrorCase;
      }
      break;
    case RunEngine:
      if ((((fabs(DegOfRad((stateGetNedToBodyEulers_f())->theta))<liftoff_pitch_limit) && (fabs(DegOfRad((stateGetNedToBodyEulers_f())->phi))<liftoff_roll_limit)) && (takeoff_timer>2))) {
        rotwing_takeoff_status = Takeoff_init;
      }
      break;
    case Takeoff_init:
      rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
      autopilot_set_in_flight(true);
      NavSetWaypointHere(WP_CLIMB);
      NavAttitude(stateGetNedToBodyEulers_f()->phi);
      NavVerticalAutoThrottleMode(stateGetNedToBodyEulers_f()->theta);
      NavVerticalThrottleMode(9600*(0.750000));
      takeoff_timer = 0.0f;
      rotwing_takeoff_status = Takeoff; 
      break;
    case Takeoff:
      rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
      NavSetWaypointHere(WP_CLIMB);
      NavVerticalAutoThrottleMode(stateGetNedToBodyEulers_f()->theta);
      NavVerticalThrottleMode(9600*(0.750000));
      if (takeoff_timer > 0.25) {
        rotwing_takeoff_status = Climb_init;
      }
      break;
    case Climb_init:
      rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
      nav_set_heading_current();
      NavGotoWaypoint(WP_CLIMB);
      NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
      NavVerticalClimbMode(nav.climb_vspeed);
      rotwing_takeoff_status = Climb;
      break;
    case Climb:
      rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
      NavGotoWaypoint(WP_CLIMB);
      NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
      NavVerticalClimbMode(nav.climb_vspeed);
      if (GetPosHeight() > 75.) {
        rotwing_state_set(ROTWING_STATE_REQUEST_FW);
        return false;
      }
      break;
    case ErrorCase:
      rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
      NavKillThrottle();
      stabilization.cmd[COMMAND_THRUST_X] = 0;
      NavAttitude(RadOfDeg(0));
      NavVerticalAutoThrottleMode(RadOfDeg(0));
      NavVerticalThrottleMode(9600*(0));
      // Stay in here forever
      break;
  }

  takeoff_timer += dt_navigation;

  return true;
}

bool nav_rotwing_land_run(void)
{
  rotwing_state_set(ROTWING_STATE_REQUEST_HOVER);
  // switch(rotwing_landing_status) {
  //   case Descend:
  //     rotwing_state_set(ROTWING_STATE_REQUEST_HOVER);
  //     NavGotoWaypoint(10);
  //     NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
  //     NavVerticalClimbMode(-(1.000000));
  //     if ((nav_block != 26) && (GetPosHeight()<12.000000)) {
  //       rotwing_landing_status = Flare;
  //     }
  //     break;
  //   case Flare:
  //     rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
  //     NavGotoWaypoint(10);
  //     NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
  //     NavVerticalClimbMode(-(0.500000));
  //     if ((nav_block != 27) && (agl_dist_valid&&(agl_dist_value<0.280000))) {
  //       rotwing_landing_status = Flarelow;
  //     }
  //     break;
  //   case Flarelow:
  //     rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
  //     NavGotoWaypoint(10);
  //     NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
  //     NavVerticalClimbMode(-(0.500000));
  //     if (((nav_block != 2) && !(nav_is_in_flight())) || ((nav_block != 2) && ground_detect())) {
  //       return false;
  //     }
  //     break; 
  // }

  // landing_timer += dt_navigation;
  return false;
}