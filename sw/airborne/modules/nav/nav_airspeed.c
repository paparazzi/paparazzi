/*
 * Copyright (C) 2014 OpenUAS
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

#include "nav_airspeed.h"
#include "generated/airframe.h"

#ifndef NOMINAL_AIRSPEED
#error Please define NOMINAL_AIRSPEED in airframe file
#endif

#ifndef TRACKING_AIRSPEED
#define TRACKING_AIRSPEED (1.25f * NOMINAL_AIRSPEED)
#endif

#ifndef LANDING_AIRSPEED
#define LANDING_AIRSPEED  (0.8f * NOMINAL_AIRSPEED)
#endif

#ifndef TAKEOFF_PITCH_ANGLE
#define TAKEOFF_PITCH_ANGLE 0
#endif


float nav_airspeed_nominal_setting = NOMINAL_AIRSPEED;
float nav_airspeed_tracking_setting = TRACKING_AIRSPEED;
float nav_airspeed_landing_setting = LANDING_AIRSPEED;

float nav_takeoff_pitch_setting = TAKEOFF_PITCH_ANGLE;

