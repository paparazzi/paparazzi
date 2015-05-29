/*
 * Copyright (C) 2009-2015 ENAC
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
 */

/**
 * @file modules/nav/nav_poles.c
 *
 */

#include "modules/nav/nav_poles.h"
#include "subsystems/navigation/common_nav.h"

uint8_t nav_poles_count = 0;
float nav_poles_time = 0.;
int8_t nav_poles_land = 1;

#define SAFETY_MARGIN 0.7

/** computes position of wp1c and wp2c, reference points for an oval around
    waypoints wp1 and wp2 */
bool nav_poles_init(uint8_t wp1, uint8_t wp2,
                    uint8_t wp1c, uint8_t wp2c,
                    float radius)
{
  float x = WaypointX(wp2) - WaypointX(wp1);
  float y = WaypointY(wp2) - WaypointY(wp1);
  float d = sqrt(x * x + y * y);

  /* Unit vector from wp1 to wp2 */
  x /= d;
  y /= d;

  WaypointX(wp2c) = WaypointX(wp2) - (x * SAFETY_MARGIN + y) * radius;
  WaypointY(wp2c) = WaypointY(wp2) - (y * SAFETY_MARGIN - x) * radius;

  WaypointX(wp1c) = WaypointX(wp1) + (x * SAFETY_MARGIN - y) * radius;
  WaypointY(wp1c) = WaypointY(wp1) + (y * SAFETY_MARGIN + x) * radius;

  return false;
}

