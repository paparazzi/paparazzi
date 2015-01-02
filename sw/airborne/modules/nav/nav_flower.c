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
 * @file modules/nav/nav_flower.c
 *
 * from OSAM advanced navigation routines
 */

#include "modules/nav/nav_flower.h"

#include "firmwares/fixedwing/nav.h"
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

bool_t nav_flower_setup(uint8_t CenterWP, uint8_t EdgeWP)
{
  Center = CenterWP;
  Edge = EdgeWP;

  EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
  EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);

  Flowerradius = sqrtf(EdgeCurrentX * EdgeCurrentX + EdgeCurrentY * EdgeCurrentY);

  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  DistanceFromCenter = sqrtf(TransCurrentX * TransCurrentX + TransCurrentY * TransCurrentY);

  FlowerTheta = atan2f(TransCurrentY, TransCurrentX);
  Fly2X = Flowerradius * cosf(FlowerTheta + 3.14) + WaypointX(Center);
  Fly2Y = Flowerradius * sinf(FlowerTheta + 3.14) + WaypointY(Center);
  FlyFromX = stateGetPositionEnu_f()->x;
  FlyFromY = stateGetPositionEnu_f()->y;

  if (DistanceFromCenter > Flowerradius) {
    CFlowerStatus = Outside;
  } else {
    CFlowerStatus = FlowerLine;
  }

  CircleX = 0;
  CircleY = 0;
  return FALSE;
}

bool_t nav_flower_run(void)
{
  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  DistanceFromCenter = sqrtf(TransCurrentX * TransCurrentX + TransCurrentY * TransCurrentY);

  bool_t InCircle = TRUE;
  float CircleTheta;

  if (DistanceFromCenter > Flowerradius) {
    InCircle = FALSE;
  }

  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(waypoints[Center].a, 0.);

  switch (CFlowerStatus) {
    case Outside:
      nav_route_xy(FlyFromX, FlyFromY, Fly2X, Fly2Y);
      if (InCircle) {
        CFlowerStatus = FlowerLine;
        FlowerTheta = atan2f(TransCurrentY, TransCurrentX);
        Fly2X = Flowerradius * cosf(FlowerTheta + 3.14) + WaypointX(Center);
        Fly2Y = Flowerradius * sinf(FlowerTheta + 3.14) + WaypointY(Center);
        FlyFromX = stateGetPositionEnu_f()->x;
        FlyFromY = stateGetPositionEnu_f()->y;
        nav_init_stage();
      }
      break;
    case FlowerLine:
      nav_route_xy(FlyFromX, FlyFromY, Fly2X, Fly2Y);
      if (!InCircle) {
        CFlowerStatus = Circle;
        CircleTheta = nav_radius / Flowerradius;
        CircleX = Flowerradius * cosf(FlowerTheta + 3.14 - CircleTheta) + WaypointX(Center);
        CircleY = Flowerradius * sinf(FlowerTheta + 3.14 - CircleTheta) + WaypointY(Center);
        nav_init_stage();
      }
      break;
    case Circle:
      nav_circle_XY(CircleX, CircleY, nav_radius);
      if (InCircle) {
        EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
        EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);
        Flowerradius = sqrtf(EdgeCurrentX * EdgeCurrentX + EdgeCurrentY * EdgeCurrentY);
        if (DistanceFromCenter > Flowerradius) {
          CFlowerStatus = Outside;
        } else {
          CFlowerStatus = FlowerLine;
        }
        FlowerTheta = atan2f(TransCurrentY, TransCurrentX);
        Fly2X = Flowerradius * cosf(FlowerTheta + 3.14) + WaypointX(Center);
        Fly2Y = Flowerradius * sinf(FlowerTheta + 3.14) + WaypointY(Center);
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
