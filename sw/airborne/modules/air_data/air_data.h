/*
 * Copyright (C) 2013 Gautier Hattenberger
 *               2014 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi
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
 * @file modules/air_data/air_data.h
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
  float airspeed_scale; ///< quadratic scale factor to convert differential pressure to airspeed

  float qnh;             ///< Barometric pressure adjusted to sea level in hPa
  float amsl_baro;       ///< altitude above sea level in m from pressure and QNH
  bool_t amsl_baro_valid; ///< TRUE if #amsl_baro is currently valid
  bool_t calc_airspeed;  ///< if TRUE, calculate airspeed from differential pressure
  bool_t calc_qnh_once;  ///< flag to calculate QNH with next pressure measurement
  bool_t calc_amsl_baro; ///< if TRUE, calculate #amsl_baro
};

/** global AirData state
 */
extern struct AirData air_data;

/** AirData initialization. Called at startup.
 */
extern void air_data_init(void);

/** Check health. Needs to be called periodically.
 */
extern void air_data_periodic(void);

/** Return AMSL (altitude AboveSeaLevel).
 * If AMSL from baro is valid, return that, otherwise from gps.
 */
extern float air_data_get_amsl(void);

extern void air_data_SetQNH(float qnh);

#endif
