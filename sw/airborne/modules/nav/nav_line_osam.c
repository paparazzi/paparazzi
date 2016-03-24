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
 * @file modules/nav/nav_line_osam.c
 *
 * Flight line from OSAM advanced navigation routines
 *
 * @todo compare with normal flight line
 */

#include "modules/nav/nav_line_osam.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

struct Point2D {float x; float y;};

enum FLStatus { FLInitialize, FLCircleS, FLLine, FLFinished };

static enum FLStatus CFLStatus = FLInitialize;
static struct Point2D FLCircle;
static struct Point2D FLFROMWP;
static struct Point2D FLTOWP;
static float FLQDR;
static float FLRadius;

/*
  Translates point so (transX, transY) are (0,0) then rotates the point around z by Zrot
*/
static void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
  float temp;

  p->x = p->x - transX;
  p->y = p->y - transY;

  temp = p->x;
  p->x = p->x * cosf(Zrot) + p->y * sinf(Zrot);
  p->y = -temp * sinf(Zrot) + p->y * cosf(Zrot);
}


bool nav_line_osam_run(uint8_t From_WP, uint8_t To_WP, float radius, float Space_Before, float Space_After)
{
  struct Point2D V;
  struct Point2D P;
  float dv;

  switch (CFLStatus) {
    case FLInitialize:

      //Translate WPs so From_WP is origin
      V.x = WaypointX(To_WP) - WaypointX(From_WP);
      V.y = WaypointY(To_WP) - WaypointY(From_WP);

      //Record Aircraft Position
      P.x = stateGetPositionEnu_f()->x;
      P.y = stateGetPositionEnu_f()->y;

      //Rotate Aircraft Position so V is aligned with x axis
      TranslateAndRotateFromWorld(&P, atan2f(V.y, V.x), WaypointX(From_WP), WaypointY(From_WP));

      //Find which side of the flight line the aircraft is on
      if (P.y > 0) {
        FLRadius = -radius;
      } else {
        FLRadius = radius;
      }

      //Find unit vector of V
      dv = sqrtf(V.x * V.x + V.y * V.y);
      V.x = V.x / dv;
      V.y = V.y / dv;

      //Find begin and end points of flight line
      FLFROMWP.x = -V.x * Space_Before;
      FLFROMWP.y = -V.y * Space_Before;

      FLTOWP.x = V.x * (dv + Space_After);
      FLTOWP.y = V.y * (dv + Space_After);

      //Find center of circle
      FLCircle.x = FLFROMWP.x + V.y * FLRadius;
      FLCircle.y = FLFROMWP.y - V.x * FLRadius;

      //Find the angle to exit the circle
      FLQDR = atan2f(FLFROMWP.x - FLCircle.x, FLFROMWP.y - FLCircle.y);

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

      if (NavCircleCount() > 0.2 && NavQdrCloseTo(DegOfRad(FLQDR))) {
        CFLStatus = FLLine;
        LINE_START_FUNCTION;
        nav_init_stage();
      }
      break;

    case FLLine:

      NavVerticalAutoThrottleMode(0); /* No pitch */
      NavVerticalAltitudeMode(waypoints[From_WP].a, 0);

      nav_route_xy(FLFROMWP.x, FLFROMWP.y, FLTOWP.x, FLTOWP.y);


      if (nav_approaching_xy(FLTOWP.x, FLTOWP.y, FLFROMWP.x, FLFROMWP.y, 0)) {
        CFLStatus = FLFinished;
        LINE_STOP_FUNCTION;
        nav_init_stage();
      }
      break;

    case FLFinished:
      CFLStatus = FLInitialize;
      nav_init_stage();
      return false;
      break;

    default:
      break;
  }
  return true;

}

static uint8_t FLBlockCount = 0;

bool nav_line_osam_block_run(uint8_t First_WP, uint8_t Last_WP, float radius, float Space_Before, float Space_After)
{
  if (First_WP < Last_WP) {
    nav_line_osam_run(First_WP + FLBlockCount, First_WP + FLBlockCount + 1, radius, Space_Before, Space_After);

    if (CFLStatus == FLInitialize) {
      FLBlockCount++;
      if (First_WP + FLBlockCount >= Last_WP) {
        FLBlockCount = 0;
        return false;
      }
    }
  } else {
    nav_line_osam_run(First_WP - FLBlockCount, First_WP - FLBlockCount - 1, radius, Space_Before, Space_After);

    if (CFLStatus == FLInitialize) {
      FLBlockCount++;
      if (First_WP - FLBlockCount <= Last_WP) {
        FLBlockCount = 0;
        return false;
      }
    }
  }

  return true;
}
