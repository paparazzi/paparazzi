/*
 * Copyright (C) 2012, Christophe De Wagter
 * Copyright (C) 2016, Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/**
 * @file modules/nav/nav_catapult.h
 * @brief catapult launch timing system
 *
 *
 * - Phase 1: Zero Roll, Climb Pitch, Zero Throttle
 * - Phase 2: After detecting the Start Acceleration\n
 *            Zero Roll, Climb Pitch, Full Throttle
 * - Phase 3: After getting the GPS heading (time based)\n
 *            Place climb in front of us\n
 *            GoTo(climb)
 */

#include "modules/nav/nav_catapult.h"

#include "state.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"


// Default values for take-off procedure

#ifndef NAV_CATAPULT_ACCELERATION_THRESHOLD
#define NAV_CATAPULT_ACCELERATION_THRESHOLD 1.5 // in g
#endif

#ifndef NAV_CATAPULT_ACCELERATION_DETECTION
#define NAV_CATAPULT_ACCELERATION_DETECTION 5   // number of valid measurements for launch detection
#endif

#ifndef NAV_CATAPULT_MOTOR_DELAY
#define NAV_CATAPULT_MOTOR_DELAY  0.75          // in seconds
#endif

#ifndef NAV_CATAPULT_HEADING_DELAY
#define NAV_CATAPULT_HEADING_DELAY 3.0          // in seconds
#endif

#ifndef NAV_CATAPULT_INITIAL_PITCH
#define NAV_CATAPULT_INITIAL_PITCH RadOfDeg(10) // in radians
#endif

#ifndef NAV_CATAPULT_INITIAL_THROTTLE
#define NAV_CATAPULT_INITIAL_THROTTLE 1.0       // [0, 1]
#endif

#ifndef NAV_CATAPULT_CLIMB_DISTANCE
#define NAV_CATAPULT_CLIMB_DISTANCE 300.        // distance of the climb point ahead of catapult
#endif

#ifndef NAV_CATAPULT_TIMEOUT
#define NAV_CATAPULT_TIMEOUT 30.                // disarm timeout (in seconds)
#endif

struct nav_catapult_struct nav_catapult;


void nav_catapult_init(void)
{

  nav_catapult.status           = NAV_CATAPULT_UNINIT;
  nav_catapult.timer            = 0;
  nav_catapult.accel_threshold  = NAV_CATAPULT_ACCELERATION_THRESHOLD;
  nav_catapult.motor_delay      = NAV_CATAPULT_MOTOR_DELAY;
  nav_catapult.heading_delay    = NAV_CATAPULT_HEADING_DELAY;
  nav_catapult.initial_pitch    = NAV_CATAPULT_INITIAL_PITCH;
  nav_catapult.initial_throttle = NAV_CATAPULT_INITIAL_THROTTLE;

}

//#############################################################
// Code that Runs in a Fast Module

void nav_catapult_highrate_module(void)
{
  if (nav_catapult.status == NAV_CATAPULT_UNINIT || nav_catapult.status == NAV_CATAPULT_ARMED) {
    nav_catapult.timer = 0;
    // nothing more to do
    return;
  }

  // increase timer
  nav_catapult.timer++;

  // wait for acceleration
  if (nav_catapult.status == NAV_CATAPULT_WAIT_ACCEL) {

    // launch detection filter
    if (nav_catapult.timer < NAV_CATAPULT_ACCELERATION_DETECTION) {
      // several consecutive measurements above threshold
#ifndef SITL
      struct FloatVect3 *accel_ned = (struct FloatVect3 *)stateGetAccelNed_f();
      struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
      struct FloatVect3 accel_body;
      float_rmat_transp_vmult(&accel_body, ned_to_body, accel_ned);
      if (accel_body.x < nav_catapult.accel_threshold * 9.81) {
        // accel is low, reset timer
        nav_catapult.timer = 0;
        return;
      }
#else
      if (autopilot.launch != true) {
        // wait for simulated launch
        nav_catapult.timer = 0;
        return;
      }
#endif
    }
    // launch was detected: Motor Delay Counter
    else if (nav_catapult.timer >= nav_catapult.motor_delay * NAV_CATAPULT_HIGHRATE_MODULE_FREQ) {
      // turn on motor
      NavVerticalThrottleMode(MAX_PPRZ * nav_catapult.initial_throttle);
      autopilot.launch = true;
      // go to next stage
      nav_catapult.status = NAV_CATAPULT_MOTOR_ON;
    }
  }

  // reaching timeout and function still running
  // shuting it down
  if (nav_catapult.timer > NAV_CATAPULT_TIMEOUT * NAV_CATAPULT_HIGHRATE_MODULE_FREQ) {
    nav_catapult.status = NAV_CATAPULT_UNINIT;
    nav_catapult_nav_catapult_highrate_module_status = MODULES_STOP;
  }
}


//############################################################
// Code that runs in 4Hz Nav

bool nav_catapult_run(uint8_t _climb)
{
  switch (nav_catapult.status) {
    case NAV_CATAPULT_UNINIT:
      // start high freq function if not done
      if (nav_catapult_nav_catapult_highrate_module_status != MODULES_RUN) {
        nav_catapult_nav_catapult_highrate_module_status = MODULES_START;
      }
      // arm catapult
      nav_catapult.status = NAV_CATAPULT_ARMED;
      break;
    case NAV_CATAPULT_ARMED:
      // store initial position
      nav_catapult.pos.x = stateGetPositionEnu_f()->x;
      nav_catapult.pos.y = stateGetPositionEnu_f()->y;
      nav_catapult.pos.z = stateGetPositionUtm_f()->alt; // useful ?
      nav_catapult.status = NAV_CATAPULT_WAIT_ACCEL;
      break;
    case NAV_CATAPULT_WAIT_ACCEL:
      // no throttle, zero attitude
      NavAttitude(RadOfDeg(0.f));
      NavVerticalAutoThrottleMode(nav_catapult.initial_pitch);
      NavVerticalThrottleMode(0.f);
      // wait for acceleration from high speed function
      break;
    case NAV_CATAPULT_MOTOR_ON:
      // fixed attitude and motor
      NavAttitude(RadOfDeg(0.f));
      NavVerticalAutoThrottleMode(nav_catapult.initial_pitch);
      NavVerticalThrottleMode(MAX_PPRZ * nav_catapult.initial_throttle);
      if (nav_catapult.timer >= nav_catapult.heading_delay * NAV_CATAPULT_HIGHRATE_MODULE_FREQ) {
        // store heading, move climb waypoint
        float dir_x = stateGetPositionEnu_f()->x - nav_catapult.pos.x;
        float dir_y = stateGetPositionEnu_f()->y - nav_catapult.pos.y;
        float dir_L = sqrtf(dir_x * dir_x + dir_y * dir_y);
        WaypointX(_climb) = nav_catapult.pos.x + (dir_x / dir_L) * NAV_CATAPULT_CLIMB_DISTANCE;
        WaypointY(_climb) = nav_catapult.pos.y + (dir_y / dir_L) * NAV_CATAPULT_CLIMB_DISTANCE;
        DownlinkSendWpNr(_climb);
        // next step
        nav_catapult.status = NAV_CATAPULT_MOTOR_CLIMB;
      }
      break;
    case NAV_CATAPULT_MOTOR_CLIMB:
      // normal climb: heading locked by waypoint target
      NavVerticalAltitudeMode(WaypointAlt(_climb), 0.f);  // vertical mode (folow glideslope)
      NavVerticalAutoThrottleMode(0.f);                   // throttle mode
      NavGotoWaypoint(_climb);                            // horizontal mode (stay on localiser)
      if (nav_approaching_xy(WaypointX(_climb), WaypointY(_climb), nav_catapult.pos.x, nav_catapult.pos.y, 0.f)) {
        // reaching climb waypoint, end procedure
        nav_catapult.status = NAV_CATAPULT_DISARM;
      }
      break;
    case NAV_CATAPULT_DISARM:
      // end procedure
      nav_catapult.status = NAV_CATAPULT_UNINIT;
      nav_catapult_nav_catapult_highrate_module_status = MODULES_STOP;
      return false;
    default:
      return false;
  }

  // procedure still running
  return true;

}

