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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file math/pprz_isa.h
 * @brief Paparazzi atmospheric pressure conversion utilities
 *
 * Conversion functions are use to approximate altitude
 * from atmospheric pressure based on the standard model
 * and the International Standard Atmosphere (ISA)
 *
 * http://en.wikipedia.org/wiki/Atmospheric_pressure
 * http://en.wikipedia.org/wiki/International_Standard_Atmosphere
 *
 * @addtogroup math
 * @{
 * @addtogroup math_isa International Standard Atmosphere utilities
 * @{
 */

#ifndef PPRZ_ISA_H
#define PPRZ_ISA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include <math.h>

// Standard Atmosphere constants
/** ISA sea level standard atmospheric pressure in Pascal */
#define PPRZ_ISA_SEA_LEVEL_PRESSURE 101325.0
/** ISA sea level standard temperature in Kelvin */
#define PPRZ_ISA_SEA_LEVEL_TEMP 288.15
/** temperature laps rate in K/m */
#define PPRZ_ISA_TEMP_LAPS_RATE 0.0065
/** earth-surface gravitational acceleration in m/s^2 */
#define PPRZ_ISA_GRAVITY 9.80665
/** universal gas constant in J/(mol*K) */
#define PPRZ_ISA_GAS_CONSTANT 8.31447
/** molar mass of dry air in kg/mol */
#define PPRZ_ISA_MOLAR_MASS 0.0289644
/** universal gas constant / molar mass of dry air in J*kg/K */
#define PPRZ_ISA_AIR_GAS_CONSTANT (PPRZ_ISA_GAS_CONSTANT/PPRZ_ISA_MOLAR_MASS)
/** standard air density in kg/m^3 */
#define PPRZ_ISA_AIR_DENSITY 1.225

static const float PPRZ_ISA_M_OF_P_CONST = (PPRZ_ISA_AIR_GAS_CONSTANT *PPRZ_ISA_SEA_LEVEL_TEMP / PPRZ_ISA_GRAVITY);

/** Convert temperature from Kelvin to Celsius */
#define CelsiusOfKelvin(_t) (_t - 274.15f)
/** Convert temperature from Celsius to Kelvin */
#define KelvinOfCelsius(_t) (_t + 274.15f)

/**
 * Get absolute altitude from pressure (using simplified equation).
 * Referrence pressure is standard pressure at sea level
 *
 * @param pressure current pressure in Pascal (Pa)
 * @return altitude in m in ISA conditions
 */
static inline float pprz_isa_altitude_of_pressure(float pressure)
{
  if (pressure > 0.) {
    return (PPRZ_ISA_M_OF_P_CONST * logf(PPRZ_ISA_SEA_LEVEL_PRESSURE / pressure));
  } else {
    return 0.;
  }
}

/**
 * Get relative altitude from pressure (using simplified equation).
 * Given the current pressure and a reference pressure (at height=0),
 * calculate the height above the reference in meters.
 * If you pass QNH as reference pressure, you get the height above sea level.
 * Using QFE as reference pressure, you get height above the airfield.
 *
 * @param pressure current pressure in Pascal (Pa)
 * @param ref_p reference pressure (QFE) when height=0 or QNH at sea level
 * @return height in m above reference in ISA conditions
 */
static inline float pprz_isa_height_of_pressure(float pressure, float ref_p)
{
  if (pressure > 0. && ref_p > 0.) {
    return (PPRZ_ISA_M_OF_P_CONST * logf(ref_p / pressure));
  } else {
    return 0.;
  }
}

/**
 * Get pressure in Pa from absolute altitude (using simplified equation).
 *
 * @param altitude current absolute altitude in meters
 * @return static pressure in Pa at given altitude in ISA conditions
 */
static inline float pprz_isa_pressure_of_altitude(float altitude)
{
  return (PPRZ_ISA_SEA_LEVEL_PRESSURE * expf((-1. / PPRZ_ISA_M_OF_P_CONST) * altitude));
}

/**
 * Get pressure in Pa from height (using simplified equation).
 *
 * @param height current height over reference (relative altitude) in meters
 * @param ref_p reference pressure (QFE or QNH) when height = 0
 * @return static pressure in Pa at given height in ISA conditions
 */
static inline float pprz_isa_pressure_of_height(float height, float ref_p)
{
  return (ref_p * expf((-1. / PPRZ_ISA_M_OF_P_CONST) * height));
}


/**
 * Get relative altitude from pressure (using full equation).
 * Given the current pressure and a reference pressure (at height=0),
 * calculate the height above the reference in meters.
 * If you pass QNH as reference pressure, you get the height above sea level.
 * Using QFE as reference pressure, you get height above the airfield.
 *
 * @param pressure current pressure in Pascal (Pa)
 * @param ref_p reference pressure (QFE or QNH) in Pa
 * @return height above reference in m in ISA conditions
 */
static inline float pprz_isa_height_of_pressure_full(float pressure, float ref_p)
{
  if (ref_p > 0.) {
    const float prel = pressure / ref_p;
    const float inv_expo = PPRZ_ISA_GAS_CONSTANT * PPRZ_ISA_TEMP_LAPS_RATE /
                           PPRZ_ISA_GRAVITY / PPRZ_ISA_MOLAR_MASS;
    return (1 - powf(prel, inv_expo)) * PPRZ_ISA_SEA_LEVEL_TEMP / PPRZ_ISA_TEMP_LAPS_RATE;
  } else {
    return 0.;
  }
}

/**
 * Get reference pressure (QFE or QNH) from current pressure and height.
 * (using full equation)
 *
 * @param pressure current pressure in Pascal (Pa)
 * @param height height above referece (sea level for QNH, airfield alt for QFE) in m
 * @return reference pressure at height=0 in Pa
 */
static inline float pprz_isa_ref_pressure_of_height_full(float pressure, float height)
{
  //  Trel = 1 - L*h/T0;
  const float Trel = 1.0 - PPRZ_ISA_TEMP_LAPS_RATE * height / PPRZ_ISA_SEA_LEVEL_TEMP;
  const float expo = PPRZ_ISA_GRAVITY * PPRZ_ISA_MOLAR_MASS / PPRZ_ISA_GAS_CONSTANT /
                     PPRZ_ISA_TEMP_LAPS_RATE;
  return pressure / pow(Trel, expo);
}

/**
 * Get ISA temperature from a MSL altitude
 *
 * @param alt AMSL altitude
 * @return temperature in ISA condition
 */
static inline float pprz_isa_temperature_of_altitude(float alt)
{
  return PPRZ_ISA_SEA_LEVEL_TEMP - PPRZ_ISA_TEMP_LAPS_RATE * alt;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_ISA_H */
/** @}*/
/** @}*/
