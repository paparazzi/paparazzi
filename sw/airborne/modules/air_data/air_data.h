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
  float pressure;     ///< Static atmospheric pressure (Pa), -1 if unknown
  float differential; ///< Differential pressure (total - static pressure) (Pa)
  float temperature;  ///< temperature in degrees Celcius, -1000 if unknown

  float airspeed;     ///< Equivalent Air Speed (equals to Calibrated Air Speed at low speed/altitude) (in m/s, -1 if unknown
  float tas;          ///< True Air Speed (TAS) in m/s, -1 if unknown
  float tas_factor;   ///< factor to convert equivalent airspeed (EAS) to true airspeed (TAS)
  float qnh;              ///< Barometric pressure adjusted to sea level in hPa, -1 if unknown
  float amsl_baro;        ///< altitude above sea level in m from pressure and QNH
  bool amsl_baro_valid; ///< TRUE if #amsl_baro is currently valid
  bool calc_airspeed;   ///< if TRUE, calculate airspeed from differential pressure
  bool calc_qnh_once;   ///< flag to calculate QNH with next pressure measurement
  bool calc_amsl_baro;  ///< if TRUE, calculate #amsl_baro
  bool calc_tas_factor; ///< if TRUE, calculate #tas_factor when getting a temp measurement

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
extern void air_data_init(void);

/** Check health. Needs to be called periodically.
 */
extern void air_data_periodic(void);

/** Return AMSL (altitude AboveSeaLevel).
 * If AMSL from baro is valid, return that, otherwise from gps.
 */
extern float air_data_get_amsl(void);

/**
 * Calculate equivalent airspeed from dynamic pressure.
 * Dynamic pressure @f$q@f$ (also called impact pressure) is the
 * difference between total(pitot) and static pressure.
 *
 * @param q dynamic pressure in Pa
 * @return equivalent airspeed in m/s
 */
extern float eas_from_dynamic_pressure(float q);

/**
 * Calculate true airspeed (TAS) factor.
 * TAS = tas_factor * EAS
 *
 * @param p current air pressure in Pa
 * @param t current air temperature in degrees Celcius
 * @return tas factor
 */
extern float get_tas_factor(float p, float t);

/**
 * Calculate true airspeed from equivalent airspeed.
 *
 * @param eas equivalent airspeed (EAS) in m/s
 * @return true airspeed in m/s
 */
extern float tas_from_eas(float eas);

/**
 * Calculate true airspeed from dynamic pressure.
 * Dynamic pressure @f$q@f$ (also called impact pressure) is the
 * difference between total(pitot) and static pressure.
 *
 * @param q dynamic pressure in Pa
 * @return true airspeed in m/s
 */
extern float tas_from_dynamic_pressure(float q);


#endif
