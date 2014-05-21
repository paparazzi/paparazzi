/*
 * Copyright (C) 2008-2013 The Paparazzi Team
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
 * @file modules/nav/nav_bungee_takeoff.c
 *
 * from OSAM advanced navigation routines
 *
 */

#include "modules/nav/nav_bungee_takeoff.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

/************** Bungee Takeoff **********************************************/

/** Takeoff functions for bungee takeoff.
    Run initialize function when the plane is on the bungee, the bungee is fully extended and you are ready to
    launch the plane. After initialized, the plane will follow a line drawn by the position of the plane on initialization and the
    position of the bungee (given in the arguments). Once the plane crosses the throttle line, which is perpendicular to the line the plane is following,
    and intersects the position of the bungee (plus or minus a fixed distance (TakeOff_Distance in airframe file) from the bungee just in case the bungee doesn't release directly above the bungee) the prop will come on. The plane will then continue to follow the line until it has reached a specific
    height (defined in as Takeoff_Height in airframe file) above the bungee waypoint and speed (defined as Takeoff_Speed in the airframe file).
    @verbatim
    <section name="Takeoff" prefix="Takeoff_">
    <define name="Height" value="30" unit="m"/>
    <define name="Speed" value="15" unit="m/s"/>
    <define name="Distance" value="10" unit="m"/>
    <define name="MinSpeed" value="5" unit="m/s"/>
    </section>
    @endverbatim
*/

#ifndef Takeoff_Distance
#define Takeoff_Distance 10
#endif
#ifndef Takeoff_Height
#define Takeoff_Height 30
#endif
#ifndef Takeoff_Speed
#define Takeoff_Speed 15
#endif
#ifndef Takeoff_MinSpeed
#define Takeoff_MinSpeed 5
#endif

enum TakeoffStatus { Launch, Throttle, Finished };
static enum TakeoffStatus CTakeoffStatus;
static float throttlePx;
static float throttlePy;
static float initialx;
static float initialy;
static float ThrottleSlope;
static bool_t AboveLine;
static float BungeeAlt;
static float TDistance;
static uint8_t BungeeWaypoint;

bool_t nav_bungee_takeoff_setup(uint8_t BungeeWP)
{
  float ThrottleB;

  initialx = stateGetPositionEnu_f()->x;
  initialy = stateGetPositionEnu_f()->y;

  BungeeWaypoint = BungeeWP;

  //Takeoff_Distance can only be positive
  TDistance = fabs(Takeoff_Distance);

  //Translate initial position so that the position of the bungee is (0,0)
  float Currentx = initialx-(WaypointX(BungeeWaypoint));
  float Currenty = initialy-(WaypointY(BungeeWaypoint));

  //Record bungee alt (which should be the ground alt at that point)
  BungeeAlt = waypoints[BungeeWaypoint].a;

  //Find Launch line slope and Throttle line slope
  float MLaunch = Currenty/Currentx;

  //Find Throttle Point (the point where the throttle line and launch line intersect)
  if(Currentx < 0)
    throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
  else
    throttlePx = -(TDistance/sqrt(MLaunch*MLaunch+1));

  if(Currenty < 0)
    throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
  else
    throttlePy = -sqrt((TDistance*TDistance)-(throttlePx*throttlePx));

  //Find ThrottleLine
  ThrottleSlope = tan(atan2(Currenty,Currentx)+(3.14/2));
  ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));

  //Determine whether the UAV is below or above the throttle line
  if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB))
    AboveLine = TRUE;
  else
    AboveLine = FALSE;

  //Enable Launch Status and turn kill throttle on
  CTakeoffStatus = Launch;
  kill_throttle = 1;

  //Translate the throttle point back
  throttlePx = throttlePx+(WaypointX(BungeeWP));
  throttlePy = throttlePy+(WaypointY(BungeeWP));

  return FALSE;
}

bool_t nav_bungee_takeoff_run(void)
{
  //Translate current position so Throttle point is (0,0)
  float Currentx = stateGetPositionEnu_f()->x-throttlePx;
  float Currenty = stateGetPositionEnu_f()->y-throttlePy;
  bool_t CurrentAboveLine;
  float ThrottleB;

  switch(CTakeoffStatus)
  {
  case Launch:
    //Follow Launch Line
    NavVerticalAutoThrottleMode(0);
    NavVerticalAltitudeMode(BungeeAlt+Takeoff_Height, 0.);
    nav_route_xy(initialx,initialy,throttlePx,throttlePy);

    kill_throttle = 1;

    //recalculate lines if below min speed
    if((*stateGetHorizontalSpeedNorm_f()) < Takeoff_MinSpeed)
    {
      initialx = stateGetPositionEnu_f()->x;
      initialy = stateGetPositionEnu_f()->y;

      //Translate initial position so that the position of the bungee is (0,0)
      Currentx = initialx-(WaypointX(BungeeWaypoint));
      Currenty = initialy-(WaypointY(BungeeWaypoint));

      //Find Launch line slope
      float MLaunch = Currenty/Currentx;

      //Find Throttle Point (the point where the throttle line and launch line intersect)
      if(Currentx < 0)
        throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
      else
        throttlePx = -(TDistance/sqrt(MLaunch*MLaunch+1));

      if(Currenty < 0)
        throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
      else
        throttlePy = -sqrt((TDistance*TDistance)-(throttlePx*throttlePx));

      //Find ThrottleLine
      ThrottleSlope = tan(atan2(Currenty,Currentx)+(3.14/2));
      ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));

      //Determine whether the UAV is below or above the throttle line
      if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB))
        AboveLine = TRUE;
      else
        AboveLine = FALSE;

      //Translate the throttle point back
      throttlePx = throttlePx+(WaypointX(BungeeWaypoint));
      throttlePy = throttlePy+(WaypointY(BungeeWaypoint));
    }

    //Find out if the UAV is currently above the line
    if(Currenty > (ThrottleSlope*Currentx))
      CurrentAboveLine = TRUE;
    else
      CurrentAboveLine = FALSE;

    //Find out if UAV has crossed the line
    if(AboveLine != CurrentAboveLine && (*stateGetHorizontalSpeedNorm_f()) > Takeoff_MinSpeed)
    {
      CTakeoffStatus = Throttle;
      kill_throttle = 0;
      nav_init_stage();
    }
    break;
  case Throttle:
    //Follow Launch Line
    NavVerticalAutoThrottleMode(AGR_CLIMB_PITCH);
    NavVerticalThrottleMode(9600*(1));
    nav_route_xy(initialx,initialy,throttlePx,throttlePy);
    kill_throttle = 0;

    if((stateGetPositionUtm_f()->alt > BungeeAlt+Takeoff_Height-10) && ((*stateGetHorizontalSpeedNorm_f()) > Takeoff_Speed))
    {
      CTakeoffStatus = Finished;
      return FALSE;
    }
    else
    {
      return TRUE;
    }
    break;
  default:
    break;
  }
  return TRUE;
}
