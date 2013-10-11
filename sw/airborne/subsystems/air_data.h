/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file subsystems/air_data.h
 * Air Data interface
 *  - pressures
 *  - airspeed
 *  - angle of attack and sideslip
 *  - wind
 */

#ifndef AIR_DATA_H
#define AIR_DATA_H

#include "std.h"

/** Air Data strucute */
struct AirData {
  float pressure;     ///< Static atmospheric pressure (Pa)
  float differential; ///< Differential pressure (dynamic - static pressure) (Pa)
  float airspeed;     ///< Conventional Air Speed (m/s)
  float aoa;          ///< angle of attack (rad)
  float sideslip;     ///< sideslip angle (rad)
  float wind_speed;   ///< wind speed (m/s)
  float wind_dir;     ///< wind direction (rad, 0 north, >0 clockwise)
};

/** global AirData state
 */
extern struct AirData air_data;

/** AirData initialization. Called at startup.
 */
extern void air_data_init( void );

#endif /* AIR_DATA_H */

