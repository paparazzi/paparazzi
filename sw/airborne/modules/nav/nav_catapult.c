/*
 *
 * Copyright (C) 2012, Christophe De Wagter
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
 * @file modules/nav/nav_catapult.h
 * @brief catapult launch timing system
 *
 *
 * Phase 1: -Zero Roll, Climb Pitch, Zero Throttle
 * Phase 2: After Feeling the Start Acceleration
 *          -Zero Roll, Climb Pitch, Full Throttle
 * Phase 3: After feeling the GPS heading (time based)
 *          -Place climb 300m in front of us
 *          -GoTo(climb)
*/



#include "generated/airframe.h"
#include "state.h"
#include "ap_downlink.h"
#include "modules/nav/nav_catapult.h"
#include "subsystems/nav.h"
#include "generated/flight_plan.h"
#include "firmwares/fixedwing/autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

// Imu is required
#include "subsystems/imu.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/datalink.h"


static bool_t nav_catapult_armed = FALSE;
static uint16_t nav_catapult_launch = 0;

#ifndef NAV_CATAPULT_ACCELERATION_THRESHOLD
#define NAV_CATAPULT_ACCELERATION_THRESHOLD 1.5;
#endif

float nav_catapult_acceleration_threshold = NAV_CATAPULT_ACCELERATION_THRESHOLD;

#ifndef NAV_CATAPULT_MOTOR_DELAY
#define NAV_CATAPULT_MOTOR_DELAY  45		// Main Control Loops
#endif

int nav_catapult_motor_delay = NAV_CATAPULT_MOTOR_DELAY;

#ifndef NAV_CATAPULT_HEADING_DELAY
#define NAV_CATAPULT_HEADING_DELAY 180
#endif

int nav_catapult_heading_delay = NAV_CATAPULT_HEADING_DELAY;

#ifndef NAV_CATAPULT_INITIAL_PITCH
#define NAV_CATAPULT_INITIAL_PITCH RadOfDeg(10)
#endif

float nav_catapult_initial_pitch = NAV_CATAPULT_INITIAL_PITCH;

#ifndef NAV_CATAPULT_INITIAL_THROTTLE
#define NAV_CATAPULT_INITIAL_THROTTLE 1.0
#endif

float nav_catapult_initial_throttle = NAV_CATAPULT_INITIAL_THROTTLE;

/////// Store Take-Off Point

static float nav_catapult_x = 0;
static float nav_catapult_y = 0;

//###############################################################################################
// Code that Runs in a Fast Module

void nav_catapult_highrate_module(void)
{
  // Only run when
  if (nav_catapult_armed)
  {
    if (nav_catapult_launch < nav_catapult_heading_delay)
      nav_catapult_launch ++;

    // Launch detection Filter
    if (nav_catapult_launch < 5)
    {
      // Five consecutive measurements > 1.5
#ifndef SITL
      struct Int32Vect3 accel_meas_body;
      INT32_RMAT_TRANSP_VMULT(accel_meas_body, imu.body_to_imu_rmat, imu.accel);
      if (ACCEL_FLOAT_OF_BFP(accel_meas_body.x)  < (nav_catapult_acceleration_threshold * 9.81))
#else
      if (launch != 1)
#endif
      {
        nav_catapult_launch = 0;
      }
    }
    // Launch was detected: Motor Delay Counter
    else if (nav_catapult_launch == nav_catapult_motor_delay)
    {
      // Turn on Motor
      NavVerticalThrottleMode(9600*(nav_catapult_initial_throttle));
      launch = 1;
    }
  }
  else
  {
    nav_catapult_launch = 0;
  }
}

//###############################################################################################
// Code that runs in 4Hz Nav

bool_t nav_catapult_init(void)
{

  nav_catapult_armed = TRUE;
  nav_catapult_launch = 0;

  return FALSE;
}



bool_t nav_catapult(uint8_t _to, uint8_t _climb)
{
  float alt = WaypointAlt(_climb);

  nav_catapult_armed = 1;

  // No Roll, Climb Pitch, No motor Phase
  if (nav_catapult_launch <= nav_catapult_motor_delay)
  {
    NavAttitude(RadOfDeg(0));
    NavVerticalAutoThrottleMode(nav_catapult_initial_pitch);
    NavVerticalThrottleMode(9600*(0));



    // Store take-off waypoint
    WaypointX(_to) = GetPosX();
    WaypointY(_to) = GetPosY();
    WaypointAlt(_to) = GetPosAlt();

    nav_catapult_x = stateGetPositionEnu_f()->x;
    nav_catapult_y = stateGetPositionEnu_f()->y;

  }
  // No Roll, Climb Pitch, Full Power
  else if (nav_catapult_launch < nav_catapult_heading_delay)
  {
    NavAttitude(RadOfDeg(0));
    NavVerticalAutoThrottleMode(nav_catapult_initial_pitch);
    NavVerticalThrottleMode(9600*(nav_catapult_initial_throttle));
  }
  // Normal Climb: Heading Locked by Waypoint Target
  else if (nav_catapult_launch == 0xffff)
  {
    NavVerticalAltitudeMode(alt, 0);	// vertical mode (folow glideslope)
    NavVerticalAutoThrottleMode(0);		// throttle mode
    NavGotoWaypoint(_climb);				// horizontal mode (stay on localiser)
  }
  else
  {
    // Store Heading, move Climb
    nav_catapult_launch = 0xffff;

    float dir_x = stateGetPositionEnu_f()->x - nav_catapult_x;
    float dir_y = stateGetPositionEnu_f()->y - nav_catapult_y;

    float dir_L = sqrt(dir_x * dir_x + dir_y * dir_y);

    WaypointX(_climb) = nav_catapult_x + (dir_x / dir_L) * 300;
    WaypointY(_climb) = nav_catapult_y + (dir_y / dir_L) * 300;

    DownlinkSendWp(DefaultChannel, DefaultDevice, _climb);
  }


return TRUE;

}	// end of gls()

bool_t nav_select_touch_down(uint8_t _td)
{
  WaypointX(_td) = GetPosX();
  WaypointY(_td) = GetPosY();
  WaypointAlt(_td) = GetPosAlt();
  return FALSE;
}



