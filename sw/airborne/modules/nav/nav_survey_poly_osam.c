/*
 * Copyright (C) 2008-2014 The Paparazzi Team
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
 * @file modules/nav/nav_survey_poly_osam.c
 *
 */

#include "modules/nav/nav_survey_poly_osam.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

#ifndef POLY_OSAM_DEFAULT_SIZE
#define POLY_OSAM_DEFAULT_SIZE 5
#endif

#ifndef POLY_OSAM_DEFAULT_SWEEP
#define POLY_OSAM_DEFAULT_SWEEP 100
#endif

/// Default entry radius, if 0 default to half sweep
#ifndef POLY_OSAM_ENTRY_RADIUS
#define POLY_OSAM_ENTRY_RADIUS 0
#endif

/// if 0 never check for min radius
#ifndef POLY_OSAM_MIN_RADIUS
#define POLY_OSAM_MIN_RADIUS 30
#endif

/// if 0 default to half sweep
#ifndef POLY_OSAM_FIRST_SWEEP_DISTANCE
#define POLY_OSAM_FIRST_SWEEP_DISTANCE 0
#endif

/// maximum number of polygon corners
#ifndef POLY_OSAM_POLYGONSIZE
#define POLY_OSAM_POLYGONSIZE 10
#endif

#ifndef POLY_OSAM_USE_FULL_CIRCLE
#define POLY_OSAM_USE_FULL_CIRCLE TRUE
#endif

// use half sweep at the end of polygon
#ifndef POLY_OSAM_HALF_SWEEP_ENABLED
#define POLY_OSAM_HALF_SWEEP_ENABLED TRUE
#endif

uint8_t Poly_Size = POLY_OSAM_DEFAULT_SIZE;
float Poly_Sweep = POLY_OSAM_DEFAULT_SWEEP;
bool use_full_circle = POLY_OSAM_USE_FULL_CIRCLE;
bool Half_Sweep_Enabled = POLY_OSAM_HALF_SWEEP_ENABLED;
bool Reset_Sweep = FALSE;

void nav_survey_poly_osam_setup_towards(uint8_t FirstWP, uint8_t Size, float Sweep, int SecondWP)
{
  float dx = waypoints[SecondWP].x - waypoints[FirstWP].x;
  float dy = waypoints[SecondWP].y - waypoints[FirstWP].y;
  if (dx == 0.0f) { dx = 0.000000001; }
  float ang = atan(dy / dx);
  //if values passed, use it.
  if (Size == 0) {Size = Poly_Size;}
  if (Sweep == 0) {Sweep = Poly_Sweep;}
  nav_survey_poly_osam_setup(FirstWP, Size, Sweep, DegOfRad(ang));
}

struct Point2D {float x; float y;};
struct Line {float m; float b; float x;};

static void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY);
static void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY);
static void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2);
static float EvaluateLineForX(float y, struct Line L);

void nav_survey_poly_osam_ResetSweepNumber(bool rst)
{
  if (rst) {
    PolySurveySweepBackNum = 0;
    Reset_Sweep = FALSE;
  }
}

#define PolygonSize POLY_OSAM_POLYGONSIZE
#define MaxFloat   1000000000
#define MinFloat   -1000000000

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

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
float EntryRadius;


void nav_survey_poly_osam_setup(uint8_t EntryWP, uint8_t Size, float sw, float Orientation)
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
  float entry_distance;

  float PolySurveyEntryDistance = POLY_OSAM_FIRST_SWEEP_DISTANCE;
  float PolySurveyEntryRadius = POLY_OSAM_ENTRY_RADIUS;

  if (PolySurveyEntryDistance == 0) {
    entry_distance = sw / 2;
  } else {
    entry_distance = PolySurveyEntryDistance;
  }

  if (PolySurveyEntryRadius == 0) {
    EntryRadius = sw / 2;
  } else {
    EntryRadius = PolySurveyEntryRadius;
  }

  SurveyTheta = RadOfDeg(Orientation);
  PolySurveySweepNum = 0;
  PolySurveySweepBackNum = 0;

  SurveyEntryWP = EntryWP;
  SurveySize = Size;
  sweep_var = sw;

  struct Point2D Corners[PolygonSize];

  CSurveyStatus = Init;

  if (Size == 0) {
    return;
  }

  //Don't initialize if Polygon is too big or if the orientation is not between 0 and 90
  if (Size <= PolygonSize && Orientation >= -90 && Orientation <= 90) {
    //Initialize Corners
    for (i = 0; i < Size; i++) {
      Corners[i].x = waypoints[i + EntryWP].x;
      Corners[i].y = waypoints[i + EntryWP].y;
    }

    //Rotate Corners so sweeps are parellel with x axis
    for (i = 0; i < Size; i++) {
      TranslateAndRotateFromWorld(&Corners[i], SurveyTheta, 0, 0);
    }

    //Find min x and min y
    SmallestCorner.y = Corners[0].y;
    SmallestCorner.x = Corners[0].x;
    for (i = 1; i < Size; i++) {
      if (Corners[i].y < SmallestCorner.y) {
        SmallestCorner.y = Corners[i].y;
      }

      if (Corners[i].x < SmallestCorner.x) {
        SmallestCorner.x = Corners[i].x;
      }
    }

    //Translate Corners all exist in quad #1
    for (i = 0; i < Size; i++) {
      TranslateAndRotateFromWorld(&Corners[i], 0, SmallestCorner.x, SmallestCorner.y);
    }

    //Rotate and Translate Entry Point
    EntryPoint.x = Corners[0].x;
    EntryPoint.y = Corners[0].y;

    //Find max y
    MaxY = Corners[0].y;
    for (i = 1; i < Size; i++) {
      if (Corners[i].y > MaxY) {
        MaxY = Corners[i].y;
      }
    }

    //Find polygon edges
    for (i = 0; i < Size; i++) {
      if (i == 0)
        if (Corners[Size - 1].x == Corners[i].x) { //Don't divide by zero!
          Edges[i].m = MaxFloat;
        } else {
          Edges[i].m = ((Corners[Size - 1].y - Corners[i].y) / (Corners[Size - 1].x - Corners[i].x));
        }
      else if (Corners[i].x == Corners[i - 1].x) {
        Edges[i].m = MaxFloat;
      } else {
        Edges[i].m = ((Corners[i].y - Corners[i - 1].y) / (Corners[i].x - Corners[i - 1].x));
      }

      //Edges[i].m = MaxFloat;
      Edges[i].b = (Corners[i].y - (Corners[i].x * Edges[i].m));
    }

    //Find Min and Max y for each line
    FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[0], Edges[1]);
    FindInterceptOfTwoLines(&temp, &RightYInt, Edges[0], Edges[Size - 1]);

    if (LeftYInt > RightYInt) {
      EdgeMaxY[0] = LeftYInt;
      EdgeMinY[0] = RightYInt;
    } else {
      EdgeMaxY[0] = RightYInt;
      EdgeMinY[0] = LeftYInt;
    }

    for (i = 1; i < Size - 1; i++) {
      FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[i], Edges[i + 1]);
      FindInterceptOfTwoLines(&temp, &RightYInt, Edges[i], Edges[i - 1]);

      if (LeftYInt > RightYInt) {
        EdgeMaxY[i] = LeftYInt;
        EdgeMinY[i] = RightYInt;
      } else {
        EdgeMaxY[i] = RightYInt;
        EdgeMinY[i] = LeftYInt;
      }
    }

    FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[Size - 1], Edges[0]);
    FindInterceptOfTwoLines(&temp, &RightYInt, Edges[Size - 1], Edges[Size - 2]);

    if (LeftYInt > RightYInt) {
      EdgeMaxY[Size - 1] = LeftYInt;
      EdgeMinY[Size - 1] = RightYInt;
    } else {
      EdgeMaxY[Size - 1] = RightYInt;
      EdgeMinY[Size - 1] = LeftYInt;
    }

    //Find amount to increment by every sweep
    if (EntryPoint.y >= MaxY / 2) {
      entry_distance = -entry_distance;
      EntryRadius = -EntryRadius;
      dSweep = -sw;
    } else {
      dSweep = sw;
    }

    //CircleQdr tells the plane when to exit the circle
    if (dSweep >= 0) {
      SurveyCircleQdr = -DegOfRad(SurveyTheta);
    } else {
      SurveyCircleQdr = 180 - DegOfRad(SurveyTheta);
    }

    //Find y value of the first sweep
    ys = EntryPoint.y + entry_distance;

    //Find the edges which intercet the sweep line first
    for (i = 0; i < SurveySize; i++) {
      if (EdgeMinY[i] <= ys && EdgeMaxY[i] > ys) {
        XIntercept2 = XIntercept1;
        XIntercept1 = EvaluateLineForX(ys, Edges[i]);
      }
    }

    //Find point to come from and point to go to
    if (fabs(EntryPoint.x - XIntercept2) <= fabs(EntryPoint.x - XIntercept1)) {
      SurveyToWP.x = XIntercept1;
      SurveyToWP.y = ys;

      SurveyFromWP.x = XIntercept2;
      SurveyFromWP.y = ys;
    } else {
      SurveyToWP.x = XIntercept2;
      SurveyToWP.y = ys;

      SurveyFromWP.x = XIntercept1;
      SurveyFromWP.y = ys;
    }

    //Find the direction to circle
    if (ys > 0 && SurveyToWP.x > SurveyFromWP.x) {
      SurveyRadius = EntryRadius;
    } else if (ys < 0 && SurveyToWP.x < SurveyFromWP.x) {
      SurveyRadius = EntryRadius;
    } else {
      SurveyRadius = -EntryRadius;
    }

    //Find the entry circle
    SurveyCircle.x = SurveyFromWP.x;
    SurveyCircle.y = EntryPoint.y + entry_distance - EntryRadius;

    //Go into entry circle state
    CSurveyStatus = Entry;
    LINE_STOP_FUNCTION;
  }
}

bool nav_survey_poly_osam_run(void)
{
  #ifdef NAV_SURVEY_POLY_OSAM_DYNAMIC
  dSweep = (nav_survey_shift > 0 ? sweep_var : -sweep_var);
  #endif
  
  struct Point2D C;
  struct Point2D ToP;
  struct Point2D FromP;
  float ys;
  static struct Point2D LastPoint;
  int i;
  bool LastHalfSweep;
  static bool HalfSweep = false;
  float XIntercept1 = 0;
  float XIntercept2 = 0;
  float DInt1 = 0;
  float DInt2 = 0;
  float temp;
  float min_radius = POLY_OSAM_MIN_RADIUS;

  if (SurveySize == 0) {
    return false;
  }

  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(waypoints[SurveyEntryWP].a, 0.);

  switch (CSurveyStatus) {
    case Entry:
      //Rotate and translate circle point into real world
      C.x = SurveyCircle.x;
      C.y = SurveyCircle.y;
      RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
      RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);

      //follow the circle
      nav_circle_XY(C.x, C.y, SurveyRadius);

      if (NavQdrCloseTo(SurveyCircleQdr) && NavCircleCountNoRewind() > .1
          && stateGetPositionUtm_f()->alt > waypoints[SurveyEntryWP].a - 10) {
        CSurveyStatus = Sweep;
        nav_init_stage();
        //LINE_START_FUNCTION;
      }
      break;
    case Sweep:
      LastHalfSweep = HalfSweep;
      ToP.x = SurveyToWP.x;
      ToP.y = SurveyToWP.y;
      FromP.x = SurveyFromWP.x;
      FromP.y = SurveyFromWP.y;

      //Rotate and Translate de plane position to local world
      C.x = stateGetPositionEnu_f()->x;
      C.y = stateGetPositionEnu_f()->y;
      TranslateAndRotateFromWorld(&C, SurveyTheta, 0, 0);
      TranslateAndRotateFromWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);

#ifdef DIGITAL_CAM
      {
        //calc distance from line start and plane position (use only X position because y can be far due to wind or other factor)
        float dist = FromP.x - C.x;

        // verify if plane are less than 10 meter from line start
        if ((dc_autoshoot == DC_AUTOSHOOT_STOP) && (fabs(dist) < 10)) {
          LINE_START_FUNCTION;
        }
      }
#endif

      //Rotate and Translate Line points into real world
      RotateAndTranslateToWorld(&ToP, 0, SmallestCorner.x, SmallestCorner.y);
      RotateAndTranslateToWorld(&ToP, SurveyTheta, 0, 0);

      RotateAndTranslateToWorld(&FromP, 0, SmallestCorner.x, SmallestCorner.y);
      RotateAndTranslateToWorld(&FromP, SurveyTheta, 0, 0);

      //follow the line
      nav_route_xy(FromP.x, FromP.y, ToP.x, ToP.y);
      if (nav_approaching_xy(ToP.x, ToP.y, FromP.x, FromP.y, 0)) {
        LastPoint.x = SurveyToWP.x;
        LastPoint.y = SurveyToWP.y;

        if (LastPoint.y + dSweep >= MaxY || LastPoint.y + dSweep <= 0) { //Your out of the Polygon so Sweep Back or Half Sweep

          if (LastPoint.y + (dSweep / 2) >= MaxY || LastPoint.y + (dSweep / 2) <= 0 || !Half_Sweep_Enabled) { //Sweep back
            dSweep = -dSweep;
            if (LastHalfSweep) {
              HalfSweep = false;
              ys = LastPoint.y + (dSweep);
            } else {
              HalfSweep = true;
              ys = LastPoint.y + (dSweep / 2);
            }

            if (dSweep >= 0) {
              SurveyCircleQdr = -DegOfRad(SurveyTheta);
            } else {
              SurveyCircleQdr = 180 - DegOfRad(SurveyTheta);
            }
            PolySurveySweepBackNum++;
          } else { // Half Sweep forward
            ys = LastPoint.y + (dSweep / 2);

            if (dSweep >= 0) {
              SurveyCircleQdr = -DegOfRad(SurveyTheta);
            } else {
              SurveyCircleQdr = 180 - DegOfRad(SurveyTheta);
            }
            HalfSweep = true;
          }


        } else { // Normal sweep
          //Find y value of the first sweep
          HalfSweep = false;
          ys = LastPoint.y + dSweep;
        }

        //Find the edges which intercet the sweep line first
        for (i = 0; i < SurveySize; i++) {
          if (EdgeMinY[i] < ys && EdgeMaxY[i] >= ys) {
            XIntercept2 = XIntercept1;
            XIntercept1 = EvaluateLineForX(ys, Edges[i]);
          }
        }

        //Find point to come from and point to go to
        DInt1 = XIntercept1 -  LastPoint.x;
        DInt2 = XIntercept2 - LastPoint.x;

        if (DInt1 * DInt2 >= 0) {
          if (fabs(DInt2) <= fabs(DInt1)) {
            SurveyToWP.x = XIntercept1;
            SurveyToWP.y = ys;

            SurveyFromWP.x = XIntercept2;
            SurveyFromWP.y = ys;
          } else {
            SurveyToWP.x = XIntercept2;
            SurveyToWP.y = ys;

            SurveyFromWP.x = XIntercept1;
            SurveyFromWP.y = ys;
          }
        } else {
          if ((SurveyToWP.x - SurveyFromWP.x) > 0 && DInt2 > 0) {
            SurveyToWP.x = XIntercept1;
            SurveyToWP.y = ys;

            SurveyFromWP.x = XIntercept2;
            SurveyFromWP.y = ys;
          } else if ((SurveyToWP.x - SurveyFromWP.x) < 0 && DInt2 < 0) {
            SurveyToWP.x = XIntercept1;
            SurveyToWP.y = ys;

            SurveyFromWP.x = XIntercept2;
            SurveyFromWP.y = ys;
          } else {
            SurveyToWP.x = XIntercept2;
            SurveyToWP.y = ys;

            SurveyFromWP.x = XIntercept1;
            SurveyFromWP.y = ys;
          }
        }

        //Find the radius to circle
        if (!HalfSweep || use_full_circle) {
          temp = dSweep / 2;
        } else {
          temp = dSweep / 4;
        }

        //if less than min radius
        if (fabs(temp) < min_radius) {
          if (temp < 0) { temp = -min_radius; } else { temp = min_radius; }
        }

        //Find the direction to circle
        if (ys > 0 && SurveyToWP.x > SurveyFromWP.x) {
          SurveyRadius = temp;
        } else if (ys < 0 && SurveyToWP.x < SurveyFromWP.x) {
          SurveyRadius = temp;
        } else {
          SurveyRadius = -temp;
        }

        //find x position to circle
        if (fabs(LastPoint.x - SurveyToWP.x) > fabs(SurveyFromWP.x - SurveyToWP.x)) {
          SurveyCircle.x = LastPoint.x;
        } else {
          SurveyCircle.x = SurveyFromWP.x;
        }

        //y position to circle
        SurveyCircle.y = ys - temp;

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

      if (NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > 0) {
        CSurveyStatus = Sweep;
        nav_init_stage();
        //LINE_START_FUNCTION;
      }
      break;
    case Init:
      return false;
    default:
      return false;
  }
  return true;
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
  p->x = p->x * cosf(Zrot) + p->y * sinf(Zrot);
  p->y = -temp * sinf(Zrot) + p->y * cosf(Zrot);
}

/// Rotates point round z by -Zrot then translates so (0,0) becomes (transX,transY)
void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
  float temp = p->x;

  p->x = p->x * cosf(Zrot) - p->y * sinf(Zrot);
  p->y = temp * sinf(Zrot) + p->y * cosf(Zrot);

  p->x = p->x + transX;
  p->y = p->y + transY;
}

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2)
{
  *x = ((L2.b - L1.b) / (L1.m - L2.m));
  *y = L1.m * (*x) + L1.b;
}


float EvaluateLineForX(float y, struct Line L)
{
  return ((y - L.b) / L.m);
}
