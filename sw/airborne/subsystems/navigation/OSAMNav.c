/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/navigation/OSAMNav.c
 *
 */

#include "subsystems/navigation/OSAMNav.h"

#include "subsystems/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

/************** Flower Navigation **********************************************/

/** Makes a flower pattern.
	CenterWP is the center of the flower. The Navigation Height is taken from this waypoint.
	EdgeWP defines the radius of the flower (distance from CenterWP to EdgeWP)
*/

enum FlowerStatus { Outside, FlowerLine, Circle };
static enum FlowerStatus CFlowerStatus;
static float CircleX;
static float CircleY;
static float Fly2X;
static float Fly2Y;
static float FlyFromX;
static float FlyFromY;
static float TransCurrentX;
static float TransCurrentY;
static float EdgeCurrentX;
static float EdgeCurrentY;
static float DistanceFromCenter;
static float FlowerTheta;
static float Flowerradius;
static uint8_t Center;
static uint8_t Edge;

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

bool_t InitializeFlower(uint8_t CenterWP, uint8_t EdgeWP)
{
  Center = CenterWP;
  Edge = EdgeWP;

  EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
  EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);

  Flowerradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);

  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  FlowerTheta = atan2(TransCurrentY,TransCurrentX);
  Fly2X = Flowerradius*cos(FlowerTheta+3.14)+WaypointX(Center);
  Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+WaypointY(Center);
  FlyFromX = stateGetPositionEnu_f()->x;
  FlyFromY = stateGetPositionEnu_f()->y;

  if(DistanceFromCenter > Flowerradius)
    CFlowerStatus = Outside;
  else
    CFlowerStatus = FlowerLine;

  CircleX = 0;
  CircleY = 0;
  return FALSE;
}

bool_t FlowerNav(void)
{
  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  bool_t InCircle = TRUE;
  float CircleTheta;

  if(DistanceFromCenter > Flowerradius)
    InCircle = FALSE;

  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(waypoints[Center].a, 0.);

  switch(CFlowerStatus)
  {
  case Outside:
    nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
    if(InCircle)
    {
      CFlowerStatus = FlowerLine;
      FlowerTheta = atan2(TransCurrentY,TransCurrentX);
      Fly2X = Flowerradius*cos(FlowerTheta+3.14)+WaypointX(Center);
      Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+WaypointY(Center);
      FlyFromX = stateGetPositionEnu_f()->x;
      FlyFromY = stateGetPositionEnu_f()->y;
      nav_init_stage();
    }
    break;
  case FlowerLine:
    nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
    if(!InCircle)
    {
      CFlowerStatus = Circle;
      CircleTheta = nav_radius/Flowerradius;
      CircleX = Flowerradius*cos(FlowerTheta+3.14-CircleTheta)+WaypointX(Center);
      CircleY = Flowerradius*sin(FlowerTheta+3.14-CircleTheta)+WaypointY(Center);
      nav_init_stage();
    }
    break;
  case Circle:
    nav_circle_XY(CircleX, CircleY, nav_radius);
    if(InCircle)
    {
      EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
      EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);
      Flowerradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);
      if(DistanceFromCenter > Flowerradius)
        CFlowerStatus = Outside;
      else
        CFlowerStatus = FlowerLine;
      FlowerTheta = atan2(TransCurrentY,TransCurrentX);
      Fly2X = Flowerradius*cos(FlowerTheta+3.14)+WaypointX(Center);
      Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+WaypointY(Center);
      FlyFromX = stateGetPositionEnu_f()->x;
      FlyFromY = stateGetPositionEnu_f()->y;
      nav_init_stage();
    }
    break;

  default:
    break;
  }
  return TRUE;
}

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

bool_t InitializeBungeeTakeoff(uint8_t BungeeWP)
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

bool_t BungeeTakeoff(void)
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

    if((stateGetPositionEnu_f()->z > BungeeAlt+Takeoff_Height-10) && ((*stateGetHorizontalSpeedNorm_f()) > Takeoff_Speed))
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

/************** Polygon Survey **********************************************/

/** This routine will cover the enitre area of any Polygon defined in the flightplan which is a convex polygon.
 */

enum SurveyStatus { Init, Entry, Sweep, SweepCircle };
static enum SurveyStatus CSurveyStatus;
static struct Point2D SmallestCorner;
static struct Line Edges[PolygonSize];
static float EdgeMaxY[PolygonSize];
static float EdgeMinY[PolygonSize];
static float SurveyTheta;
static float dSweep;
static float SurveyRadius;
static struct Point2D SurveyToWP;
static struct Point2D SurveyFromWP;
static struct Point2D SurveyCircle;
static uint8_t SurveyEntryWP;
static uint8_t SurveySize;
static float SurveyCircleQdr;
static float MaxY;
uint16_t PolySurveySweepNum;
uint16_t PolySurveySweepBackNum;

bool_t InitializePolygonSurvey(uint8_t EntryWP, uint8_t Size, float sw, float Orientation)
{
  SmallestCorner.x = 0;
  SmallestCorner.y = 0;
  int i = 0;
  float ys = 0;
  static struct Point2D EntryPoint;
  float LeftYInt;
  float RightYInt;
  float temp;
  float XIntercept1 = 0;
  float XIntercept2 = 0;

  SurveyTheta = RadOfDeg(Orientation);
  PolySurveySweepNum = 0;
  PolySurveySweepBackNum = 0;

  SurveyEntryWP = EntryWP;
  SurveySize = Size;

  struct Point2D Corners[PolygonSize];

  CSurveyStatus = Init;

  if (Size == 0)
    return TRUE;

  //Don't initialize if Polygon is too big or if the orientation is not between 0 and 90
  if(Size <= PolygonSize && Orientation >= -90 && Orientation <= 90)
  {
    //Initialize Corners
    for(i = 0; i < Size; i++)
    {
      Corners[i].x = waypoints[i+EntryWP].x;
      Corners[i].y = waypoints[i+EntryWP].y;
    }

    //Rotate Corners so sweeps are parellel with x axis
    for(i = 0; i < Size; i++)
      TranslateAndRotateFromWorld(&Corners[i], SurveyTheta, 0, 0);

    //Find min x and min y
    SmallestCorner.y = Corners[0].y;
    SmallestCorner.x = Corners[0].x;
    for(i = 1; i < Size; i++)
    {
      if(Corners[i].y < SmallestCorner.y)
        SmallestCorner.y = Corners[i].y;

      if(Corners[i].x < SmallestCorner.x)
        SmallestCorner.x = Corners[i].x;
    }

    //Translate Corners all exist in quad #1
    for(i = 0; i < Size; i++)
      TranslateAndRotateFromWorld(&Corners[i], 0, SmallestCorner.x, SmallestCorner.y);

    //Rotate and Translate Entry Point
    EntryPoint.x = Corners[0].x;
    EntryPoint.y = Corners[0].y;

    //Find max y
    MaxY = Corners[0].y;
    for(i = 1; i < Size; i++)
    {
      if(Corners[i].y > MaxY)
        MaxY = Corners[i].y;
    }

    //Find polygon edges
    for(i = 0; i < Size; i++)
    {
      if(i == 0)
        if(Corners[Size-1].x == Corners[i].x) //Don't divide by zero!
          Edges[i].m = MaxFloat;
        else
          Edges[i].m = ((Corners[Size-1].y-Corners[i].y)/(Corners[Size-1].x-Corners[i].x));
      else
        if(Corners[i].x == Corners[i-1].x)
          Edges[i].m = MaxFloat;
        else
          Edges[i].m = ((Corners[i].y-Corners[i-1].y)/(Corners[i].x-Corners[i-1].x));

      //Edges[i].m = MaxFloat;
      Edges[i].b = (Corners[i].y - (Corners[i].x*Edges[i].m));
    }

    //Find Min and Max y for each line
    FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[0], Edges[1]);
    FindInterceptOfTwoLines(&temp, &RightYInt, Edges[0], Edges[Size-1]);

    if(LeftYInt > RightYInt)
    {
      EdgeMaxY[0] = LeftYInt;
      EdgeMinY[0] = RightYInt;
    }
    else
    {
      EdgeMaxY[0] = RightYInt;
      EdgeMinY[0] = LeftYInt;
    }

    for(i = 1; i < Size-1; i++)
    {
      FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[i], Edges[i+1]);
      FindInterceptOfTwoLines(&temp, &RightYInt, Edges[i], Edges[i-1]);

      if(LeftYInt > RightYInt)
      {
        EdgeMaxY[i] = LeftYInt;
        EdgeMinY[i] = RightYInt;
      }
      else
      {
        EdgeMaxY[i] = RightYInt;
        EdgeMinY[i] = LeftYInt;
      }
    }

    FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[Size-1], Edges[0]);
    FindInterceptOfTwoLines(&temp, &RightYInt, Edges[Size-1], Edges[Size-2]);

    if(LeftYInt > RightYInt)
    {
      EdgeMaxY[Size-1] = LeftYInt;
      EdgeMinY[Size-1] = RightYInt;
    }
    else
    {
      EdgeMaxY[Size-1] = RightYInt;
      EdgeMinY[Size-1] = LeftYInt;
    }

    //Find amount to increment by every sweep
    if(EntryPoint.y >= MaxY/2)
      dSweep = -sw;
    else
      dSweep = sw;

    //CircleQdr tells the plane when to exit the circle
    if(dSweep >= 0)
      SurveyCircleQdr = -DegOfRad(SurveyTheta);
    else
      SurveyCircleQdr = 180-DegOfRad(SurveyTheta);

    //Find y value of the first sweep
    ys = EntryPoint.y+(dSweep/2);

    //Find the edges which intercet the sweep line first
    for(i = 0; i < SurveySize; i++)
    {
      if(EdgeMinY[i] <= ys && EdgeMaxY[i] > ys)
      {
        XIntercept2 = XIntercept1;
        XIntercept1 = EvaluateLineForX(ys, Edges[i]);
      }
    }

    //Find point to come from and point to go to
    if(fabs(EntryPoint.x - XIntercept2) <= fabs(EntryPoint.x - XIntercept1))
    {
      SurveyToWP.x = XIntercept1;
      SurveyToWP.y = ys;

      SurveyFromWP.x = XIntercept2;
      SurveyFromWP.y = ys;
    }
    else
    {
      SurveyToWP.x = XIntercept2;
      SurveyToWP.y = ys;

      SurveyFromWP.x = XIntercept1;
      SurveyFromWP.y = ys;
    }

    //Find the direction to circle
    if(ys > 0 && SurveyToWP.x > SurveyFromWP.x)
      SurveyRadius = dSweep/2;
    else if(ys < 0 && SurveyToWP.x < SurveyFromWP.x)
      SurveyRadius = dSweep/2;
    else
      SurveyRadius = -dSweep/2;

    //Find the entry circle
    SurveyCircle.x = SurveyFromWP.x;
    SurveyCircle.y = EntryPoint.y;

    //Go into entry circle state
    CSurveyStatus = Entry;
    LINE_STOP_FUNCTION;
  }

  return FALSE;
}

bool_t PolygonSurvey(void)
{
  struct Point2D C;
  struct Point2D ToP;
  struct Point2D FromP;
  float ys;
  static struct Point2D LastPoint;
  int i;
  bool_t SweepingBack = FALSE;
  float XIntercept1 = 0;
  float XIntercept2 = 0;
  float DInt1 = 0;
  float DInt2 = 0;

  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(waypoints[SurveyEntryWP].a, 0.);

  switch(CSurveyStatus)
  {
  case Entry:
    //Rotate and translate circle point into real world
    C.x = SurveyCircle.x;
    C.y = SurveyCircle.y;
    RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);

    //follow the circle
    nav_circle_XY(C.x, C.y, SurveyRadius);

    if(NavQdrCloseTo(SurveyCircleQdr) && NavCircleCountNoRewind() > .1 && stateGetPositionEnu_f()->z > waypoints[SurveyEntryWP].a-10)
    {
      CSurveyStatus = Sweep;
      nav_init_stage();
      LINE_START_FUNCTION;
    }
    break;
  case Sweep:
    //Rotate and Translate Line points into real world
    ToP.x = SurveyToWP.x;
    ToP.y = SurveyToWP.y;
    FromP.x = SurveyFromWP.x;
    FromP.y = SurveyFromWP.y;

    RotateAndTranslateToWorld(&ToP, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&ToP, SurveyTheta, 0, 0);

    RotateAndTranslateToWorld(&FromP, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&FromP, SurveyTheta, 0, 0);

    //follow the line
    nav_route_xy(FromP.x,FromP.y,ToP.x,ToP.y);
    if(nav_approaching_xy(ToP.x,ToP.y,FromP.x,FromP.y, 0))
    {
      LastPoint.x = SurveyToWP.x;
      LastPoint.y = SurveyToWP.y;

      if(LastPoint.y+dSweep >= MaxY || LastPoint.y+dSweep <= 0) //Your out of the Polygon so Sweep Back
      {
        dSweep = -dSweep;
        ys = LastPoint.y+(dSweep/2);

        if(dSweep >= 0)
          SurveyCircleQdr = -DegOfRad(SurveyTheta);
        else
          SurveyCircleQdr = 180-DegOfRad(SurveyTheta);
        SweepingBack = TRUE;
        PolySurveySweepBackNum++;
      }
      else
      {
        //Find y value of the first sweep
        ys = LastPoint.y+dSweep;
      }

      //Find the edges which intercet the sweep line first
      for(i = 0; i < SurveySize; i++)
      {
        if(EdgeMinY[i] < ys && EdgeMaxY[i] >= ys)
        {
          XIntercept2 = XIntercept1;
          XIntercept1 = EvaluateLineForX(ys, Edges[i]);
        }
      }

      //Find point to come from and point to go to
      DInt1 = XIntercept1 -  LastPoint.x;
      DInt2 = XIntercept2 - LastPoint.x;

      if(DInt1 * DInt2 >= 0)
      {
        if(fabs(DInt2) <= fabs(DInt1))
        {
          SurveyToWP.x = XIntercept1;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept2;
          SurveyFromWP.y = ys;
        }
        else
        {
          SurveyToWP.x = XIntercept2;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept1;
          SurveyFromWP.y = ys;
        }
      }
      else
      {
        if((SurveyToWP.x - SurveyFromWP.x) > 0 && DInt2 > 0)
        {
          SurveyToWP.x = XIntercept1;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept2;
          SurveyFromWP.y = ys;
        }
        else if((SurveyToWP.x - SurveyFromWP.x) < 0 && DInt2 < 0)
        {
          SurveyToWP.x = XIntercept1;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept2;
          SurveyFromWP.y = ys;
        }
        else
        {
          SurveyToWP.x = XIntercept2;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept1;
          SurveyFromWP.y = ys;
        }
      }



      if(fabs(LastPoint.x-SurveyToWP.x) > fabs(SurveyFromWP.x-SurveyToWP.x))
        SurveyCircle.x = LastPoint.x;
      else
        SurveyCircle.x = SurveyFromWP.x;


      if(!SweepingBack)
        SurveyCircle.y = LastPoint.y+(dSweep/2);
      else
        SurveyCircle.y = LastPoint.y;

      //Find the direction to circle
      if(ys > 0 && SurveyToWP.x > SurveyFromWP.x)
        SurveyRadius = dSweep/2;
      else if(ys < 0 && SurveyToWP.x < SurveyFromWP.x)
        SurveyRadius = dSweep/2;
      else
        SurveyRadius = -dSweep/2;

      //Go into circle state
      CSurveyStatus = SweepCircle;
      nav_init_stage();
      LINE_STOP_FUNCTION;
      PolySurveySweepNum++;
    }

    break;
  case SweepCircle:
    //Rotate and translate circle point into real world
    C.x = SurveyCircle.x;
    C.y = SurveyCircle.y;
    RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);

    //follow the circle
    nav_circle_XY(C.x, C.y, SurveyRadius);

    if(NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > 0)
    {
      CSurveyStatus = Sweep;
      nav_init_stage();
      LINE_START_FUNCTION;
    }
    break;
  case Init:
    return FALSE;
  default:
    return FALSE;
  }
  return TRUE;
}

/************** Vertical Raster **********************************************/

/** Copy of nav line. The only difference is it changes altitude every sweep, but doesn't come out of circle until
    it reaches altitude.
*/
enum line_status { LR12, LQC21, LTC2, LQC22, LR21, LQC12, LTC1, LQC11 };
static enum line_status line_status;

bool_t InitializeVerticalRaster( void ) {
  line_status = LR12;
  return FALSE;
}

bool_t VerticalRaster(uint8_t l1, uint8_t l2, float radius, float AltSweep) {
  radius = fabs(radius);
  float alt = waypoints[l1].a;
  waypoints[l2].a = alt;

  float l2_l1_x = WaypointX(l1) - WaypointX(l2);
  float l2_l1_y = WaypointY(l1) - WaypointY(l2);
  float d = sqrt(l2_l1_x*l2_l1_x+l2_l1_y*l2_l1_y);

  /* Unit vector from l1 to l2 */
  float u_x = l2_l1_x / d;
  float u_y = l2_l1_y / d;

  /* The half circle centers and the other leg */
  struct point l2_c1 = { WaypointX(l1) + radius * u_y,
                         WaypointY(l1) + radius * -u_x,
                         alt  };
  struct point l2_c2 = { WaypointX(l1) + 1.732*radius * u_x,
                         WaypointY(l1) + 1.732*radius * u_y,
                         alt  };
  struct point l2_c3 = { WaypointX(l1) + radius * -u_y,
                         WaypointY(l1) + radius * u_x,
                         alt  };

  struct point l1_c1 = { WaypointX(l2) + radius * -u_y,
                         WaypointY(l2) + radius * u_x,
                         alt  };
  struct point l1_c2 = { WaypointX(l2) +1.732*radius * -u_x,
                         WaypointY(l2) + 1.732*radius * -u_y,
                         alt  };
  struct point l1_c3 = { WaypointX(l2) + radius * u_y,
                         WaypointY(l2) + radius * -u_x,
                         alt  };
  float qdr_out_2_1 = M_PI/3. - atan2(u_y, u_x);

  float qdr_out_2_2 = -M_PI/3. - atan2(u_y, u_x);
  float qdr_out_2_3 = M_PI - atan2(u_y, u_x);

  /* Vertical target */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(WaypointAlt(l1), 0.);

  switch (line_status) {
  case LR12: /* From wp l2 to wp l1 */
    NavSegment(l2, l1);
    if (NavApproachingFrom(l1, l2, CARROT)) {
      line_status = LQC21;
      waypoints[l1].a = waypoints[l1].a+AltSweep;
      nav_init_stage();
    }
    break;
  case LQC21:
    nav_circle_XY(l2_c1.x, l2_c1.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_1)-10)) {
      line_status = LTC2;
      nav_init_stage();
    }
    break;
  case LTC2:
    nav_circle_XY(l2_c2.x, l2_c2.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_2)+10) && stateGetPositionEnu_f()->z >= (waypoints[l1].a-10)) {
      line_status = LQC22;
      nav_init_stage();
    }
    break;
  case LQC22:
    nav_circle_XY(l2_c3.x, l2_c3.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_3)-10)) {
      line_status = LR21;
      nav_init_stage();
    }
    break;
  case LR21: /* From wp l1 to wp l2 */
    NavSegment(l1, l2);
    if (NavApproachingFrom(l2, l1, CARROT)) {
      line_status = LQC12;
      waypoints[l1].a = waypoints[l1].a+AltSweep;
      nav_init_stage();
    }
    break;
  case LQC12:
    nav_circle_XY(l1_c1.x, l1_c1.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_1 + M_PI)-10)) {
      line_status = LTC1;
      nav_init_stage();
    }
    break;
  case LTC1:
    nav_circle_XY(l1_c2.x, l1_c2.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_2 + M_PI)+10) && stateGetPositionEnu_f()->z >= (waypoints[l1].a-5)) {
      line_status = LQC11;
      nav_init_stage();
    }
    break;
  case LQC11:
    nav_circle_XY(l1_c3.x, l1_c3.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_3 + M_PI)-10)) {
      line_status = LR12;
      nav_init_stage();
    }
  default:
    break;
  }
  return TRUE; /* This pattern never ends */
}

/************** SkidLanding **********************************************/
/**
   Landing Routine

   @verbatim
   <section name="Landing" prefix="Landing_">
   <define name="AFHeight" value="50" unit="m"/>
   <define name="FinalHeight" value="5" unit="m"/>
   <define name="FinalStageTime" value="5" unit="s"/>
   </section>
   @endverbatim
*/

#ifndef Landing_AFHeight
#define Landing_AFHeight 50
#endif
#ifndef Landing_FinalHeight
#define Landing_FinalHeight 5
#endif
#ifndef Landing_FinalStageTime
#define Landing_FinalStageTime 5
#endif

enum LandingStatus { CircleDown, LandingWait, Final, Approach };
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

bool_t InitializeSkidLanding(uint8_t AFWP, uint8_t TDWP, float radius)
{
  AFWaypoint = AFWP;
  TDWaypoint = TDWP;
  CLandingStatus = CircleDown;
  LandRadius = radius;
  LandAppAlt = stateGetPositionEnu_f()->z;
  FinalLandAltitude = Landing_FinalHeight;
  FinalLandCount = 1;
  waypoints[AFWaypoint].a = waypoints[TDWaypoint].a + Landing_AFHeight;

  float x_0 = WaypointX(TDWaypoint) - WaypointX(AFWaypoint);
  float y_0 = WaypointY(TDWaypoint) - WaypointY(AFWaypoint);

  /* Unit vector from AF to TD */
  float d = sqrt(x_0*x_0+y_0*y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  LandCircle.x = WaypointX(AFWaypoint) + y_1 * LandRadius;
  LandCircle.y = WaypointY(AFWaypoint) - x_1 * LandRadius;

  LandCircleQDR = atan2(WaypointX(AFWaypoint)-LandCircle.x, WaypointY(AFWaypoint)-LandCircle.y);

  if(LandRadius > 0)
  {
    ApproachQDR = LandCircleQDR-RadOfDeg(90);
    LandCircleQDR = LandCircleQDR-RadOfDeg(45);
  }
  else
  {
    ApproachQDR = LandCircleQDR+RadOfDeg(90);
    LandCircleQDR = LandCircleQDR+RadOfDeg(45);
  }


  return FALSE;
}

bool_t SkidLanding(void)
{
  switch(CLandingStatus)
  {
  case CircleDown:
    NavVerticalAutoThrottleMode(0); /* No pitch */

    if(NavCircleCount() < .1)
    {
      NavVerticalAltitudeMode(LandAppAlt, 0);
    }
    else
      NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);

    nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

    if(stateGetPositionEnu_f()->z < waypoints[AFWaypoint].a + 5)
    {
      CLandingStatus = LandingWait;
      nav_init_stage();
    }

    break;

  case LandingWait:
    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
    nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

    if(NavCircleCount() > 0.5 && NavQdrCloseTo(DegOfRad(ApproachQDR)))
    {
      CLandingStatus = Approach;
      nav_init_stage();
    }
    break;

  case Approach:
    kill_throttle = 1;
    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
    nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

    if(NavQdrCloseTo(DegOfRad(LandCircleQDR)))
    {
      CLandingStatus = Final;
      nav_init_stage();
    }
    break;

  case Final:
    kill_throttle = 1;
    NavVerticalAutoThrottleMode(0);
    NavVerticalAltitudeMode(waypoints[TDWaypoint].a+FinalLandAltitude, 0);
    nav_route_xy(WaypointX(AFWaypoint),WaypointY(AFWaypoint),WaypointX(TDWaypoint),WaypointY(TDWaypoint));
    if(stage_time >= Landing_FinalStageTime*FinalLandCount)
    {
      FinalLandAltitude = FinalLandAltitude/2;
      FinalLandCount++;
    }
    break;

  default:

    break;
  }
  return TRUE;
}

enum FLStatus { FLInitialize, FLCircleS, FLLine, FLFinished };
static enum FLStatus CFLStatus = FLInitialize;
static struct Point2D FLCircle;
static struct Point2D FLFROMWP;
static struct Point2D FLTOWP;
static float FLQDR;
static float FLRadius;

bool_t FlightLine(uint8_t From_WP, uint8_t To_WP, float radius, float Space_Before, float Space_After)
{
  struct Point2D V;
  struct Point2D P;
  float dv;

  switch(CFLStatus)
  {
  case FLInitialize:

    //Translate WPs so From_WP is origin
    V.x = WaypointX(To_WP) - WaypointX(From_WP);
    V.y = WaypointY(To_WP) - WaypointY(From_WP);

    //Record Aircraft Position
    P.x = stateGetPositionEnu_f()->x;
    P.y = stateGetPositionEnu_f()->y;

    //Rotate Aircraft Position so V is aligned with x axis
    TranslateAndRotateFromWorld(&P, atan2(V.y,V.x), WaypointX(From_WP), WaypointY(From_WP));

    //Find which side of the flight line the aircraft is on
    if(P.y > 0)
      FLRadius = -radius;
    else
      FLRadius = radius;

    //Find unit vector of V
    dv = sqrt(V.x*V.x+V.y*V.y);
    V.x = V.x / dv;
    V.y = V.y / dv;

    //Find begin and end points of flight line
    FLFROMWP.x = -V.x*Space_Before;
    FLFROMWP.y = -V.y*Space_Before;

    FLTOWP.x = V.x*(dv+Space_After);
    FLTOWP.y = V.y*(dv+Space_After);

    //Find center of circle
    FLCircle.x = FLFROMWP.x + V.y * FLRadius;
    FLCircle.y = FLFROMWP.y - V.x * FLRadius;

    //Find the angle to exit the circle
    FLQDR = atan2(FLFROMWP.x-FLCircle.x, FLFROMWP.y-FLCircle.y);

    //Translate back
    FLFROMWP.x = FLFROMWP.x + WaypointX(From_WP);
    FLFROMWP.y = FLFROMWP.y + WaypointY(From_WP);

    FLTOWP.x = FLTOWP.x + WaypointX(From_WP);
    FLTOWP.y = FLTOWP.y + WaypointY(From_WP);

    FLCircle.x = FLCircle.x + WaypointX(From_WP);
    FLCircle.y = FLCircle.y + WaypointY(From_WP);

    CFLStatus = FLCircleS;
    nav_init_stage();

    break;

  case FLCircleS:

    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[From_WP].a, 0);

    nav_circle_XY(FLCircle.x, FLCircle.y, FLRadius);

    if(NavCircleCount() > 0.2 && NavQdrCloseTo(DegOfRad(FLQDR)))
    {
      CFLStatus = FLLine;
      LINE_START_FUNCTION;
      nav_init_stage();
    }
    break;

  case FLLine:

    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[From_WP].a, 0);

    nav_route_xy(FLFROMWP.x,FLFROMWP.y,FLTOWP.x,FLTOWP.y);


    if(nav_approaching_xy(FLTOWP.x,FLTOWP.y,FLFROMWP.x,FLFROMWP.y, 0))
    {
      CFLStatus = FLFinished;
      LINE_STOP_FUNCTION;
      nav_init_stage();
    }
    break;

  case FLFinished:
    CFLStatus = FLInitialize;
    nav_init_stage();
    return FALSE;
    break;

  default:
    break;
  }
  return TRUE;

}

static uint8_t FLBlockCount = 0;

bool_t FlightLineBlock(uint8_t First_WP, uint8_t Last_WP, float radius, float Space_Before, float Space_After)
{
  if(First_WP < Last_WP)
  {
    FlightLine(First_WP+FLBlockCount, First_WP+FLBlockCount+1, radius, Space_Before, Space_After);

    if(CFLStatus == FLInitialize)
    {
      FLBlockCount++;
      if(First_WP+FLBlockCount >= Last_WP)
      {
        FLBlockCount = 0;
        return FALSE;
      }
    }
  }
  else
  {
    FlightLine(First_WP-FLBlockCount, First_WP-FLBlockCount-1, radius, Space_Before, Space_After);

    if(CFLStatus == FLInitialize)
    {
      FLBlockCount++;
      if(First_WP-FLBlockCount <= Last_WP)
      {
        FLBlockCount = 0;
        return FALSE;
      }
    }
  }

  return TRUE;
}


/*
  Translates point so (transX, transY) are (0,0) then rotates the point around z by Zrot
*/
void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
  float temp;

  p->x = p->x - transX;
  p->y = p->y - transY;

  temp = p->x;
  p->x = p->x*cos(Zrot)+p->y*sin(Zrot);
  p->y = -temp*sin(Zrot)+p->y*cos(Zrot);
}

/// Rotates point round z by -Zrot then translates so (0,0) becomes (transX,transY)
void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
  float temp = p->x;

  p->x = p->x*cos(Zrot)-p->y*sin(Zrot);
  p->y = temp*sin(Zrot)+p->y*cos(Zrot);

  p->x = p->x + transX;
  p->y = p->y + transY;
}

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2)
{
  *x = ((L2.b-L1.b)/(L1.m-L2.m));
  *y = L1.m*(*x)+L1.b;
}

float EvaluateLineForY(float x, struct Line L)
{
  return (L.m*x)+L.b;
}

float EvaluateLineForX(float y, struct Line L)
{
  return ((y-L.b)/L.m);
}

float DistanceEquation(struct Point2D p1,struct Point2D p2)
{
  return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}
