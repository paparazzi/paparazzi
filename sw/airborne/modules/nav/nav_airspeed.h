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

/** \file nav_airspeed.h
 *
 * NAV Tunable standard airspeed settings to be called from the flight plan
 */

#ifndef NAV_AIRSPEED_H
#define NAV_AIRSPEED_H

#include "std.h"

extern float nav_airspeed_nominal_setting;
extern float nav_airspeed_tracking_setting;
extern float nav_airspeed_landing_setting;
extern float nav_takeoff_pitch_setting;
#endif
