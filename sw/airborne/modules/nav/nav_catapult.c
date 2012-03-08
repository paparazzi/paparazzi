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
 * @file module/nav/nav_catapult.h
 * @brief catapult launch timing system
 *
*/



#include "generated/airframe.h"
#include "estimator.h"
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
#include "downlink.h"


static bool_t nav_catapult_armed = FALSE;
static uint16_t nav_catapult_launch = 0;

#ifndef NAV_CATAPULT_ACCELERATION_THRESHOLD
#define NAV_CATAPULT_ACCELERATION_THRESHOLD (1.5 * 9.81);
#endif

#ifndef NAV_CATAPULT_MOTOR_DELAY
#define NAV_CATAPULT_MOTOR_DELAY  20		// Main Control Loops
#endif

#define NAV_CATAPULT_HEADING_DELAY (60 * 3)

static float nav_catapult_x = 0;
static float nav_catapult_y = 0;

//###############################################################################################
// Code that Runs in a Fast Module

void nav_catapult_highrate_module(void)
{
  // Only run when
  if (nav_catapult_armed)
  {
    if (nav_catapult_launch < NAV_CATAPULT_HEADING_DELAY)
      nav_catapult_launch ++;

    // Launch detection Filter
    if (nav_catapult_launch < 5)
    {
      // Five consecutive measurements > 1.5
#ifndef SITL
      if (ACCEL_FLOAT_OF_BFP(imu.accel.x)  < (1.5f * 9.1))
#else
      if (launch != 1)
#endif
      {
        nav_catapult_launch = 0;
      }
    }
    // Launch was detected: Motor Delay Counter
    else if (nav_catapult_launch == NAV_CATAPULT_MOTOR_DELAY)
    {
      // Turn on Motor
      NavVerticalThrottleMode(9600*(1));
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



bool_t nav_catapult(uint8_t _climb) 
{
  float alt = WaypointAlt(_climb);

  nav_catapult_armed = 1;

/*

  float nav_final_progress = ((estimator_x - WaypointX(_tod)) * final_x + (estimator_y - WaypointY(_tod)) * final_y) / final2;
  Bound(nav_final_progress,-1,1);
  float nav_final_length = sqrt(final2);

  float pre_climb = -(WaypointAlt(_tod) - WaypointAlt(_td)) / (nav_final_length / estimator_hspeed_mod);
  Bound(pre_climb, -5, 0.);

  float start_alt = WaypointAlt(_tod);
  float diff_alt = WaypointAlt(_td) - start_alt;
  float alt = start_alt + nav_final_progress * diff_alt;
  Bound(alt, WaypointAlt(_td), start_alt +(pre_climb/(v_ctl_altitude_pgain))) // to prevent climbing before intercept

*/

  // No Roll, Climb Pitch, No motor Phase
  if (nav_catapult_launch <= NAV_CATAPULT_MOTOR_DELAY)
  {
    NavAttitude(RadOfDeg(0));
    NavVerticalAutoThrottleMode(RadOfDeg(15));
    NavVerticalThrottleMode(9600*(0));

    // Store take-off waypoint
    nav_catapult_x = estimator_x;
    nav_catapult_y = estimator_y;

  }
  // No Roll, Climb Pitch, Full Power
  else if (nav_catapult_launch < NAV_CATAPULT_HEADING_DELAY)
  {
    NavAttitude(RadOfDeg(0));
    NavVerticalAutoThrottleMode(RadOfDeg(15));
    NavVerticalThrottleMode(9600*(1.0));
  }
  // Heading Lock
  else if (nav_catapult_launch == 0xffff)
  {
    NavVerticalAltitudeMode(alt, 0);	// vertical mode (folow glideslope)
    NavVerticalAutoThrottleMode(RadOfDeg(15));		// throttle mode
    NavGotoWaypoint(_climb);				// horizontal mode (stay on localiser)
  }
  else
  {
    // Store Heading, move Climb
    nav_catapult_launch = 0xffff;

    float dir_x = estimator_x - nav_catapult_x;
    float dir_y = estimator_y - nav_catapult_y;

    float dir_L = sqrt(dir_x * dir_x + dir_y * dir_y);

    WaypointX(_climb) = nav_catapult_x + (dir_x / dir_L) * 300;
    WaypointY(_climb) = nav_catapult_y + (dir_y / dir_L) * 300;

    DownlinkSendWp(DefaultChannel, _climb);
  }


return TRUE;

}	// end of gls()
