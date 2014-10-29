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
#define TRACKING_AIRSPEED NOMINAL_AIRSPEED
#endif

#ifndef GLIDE_AIRSPEED
#define GLIDE_AIRSPEED NOMINAL_AIRSPEED
#endif

#ifndef TAKEOFF_PITCH_ANGLE
#define TAKEOFF_PITCH_ANGLE 0
#endif

#ifndef FLARE_PITCH_ANGLE
#define FLARE_PITCH_ANGLE 0
#endif

float set_airspeed_nominal = NOMINAL_AIRSPEED;
float set_airspeed_tracking = TRACKING_AIRSPEED;
float set_airspeed_glide = GLIDE_AIRSPEED;
float set_takeoff_pitch = TAKEOFF_PITCH_ANGLE;
float set_flare_pitch = FLARE_PITCH_ANGLE;

