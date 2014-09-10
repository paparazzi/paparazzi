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
 */

#ifndef PPRZ_ISA_H
#define PPRZ_ISA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include <math.h>

// Standard Atmosphere constants
#define PPRZ_ISA_SEA_LEVEL_PRESSURE 101325.0
#define PPRZ_ISA_SEA_LEVEL_TEMP 288.15
#define PPRZ_ISA_TEMP_LAPS_RATE 0.0065
#define PPRZ_ISA_GRAVITY 9.80665
#define PPRZ_ISA_AIR_GAS_CONSTANT (8.31447/0.0289644)

static const float PPRZ_ISA_M_OF_P_CONST = (PPRZ_ISA_AIR_GAS_CONSTANT* PPRZ_ISA_SEA_LEVEL_TEMP / PPRZ_ISA_GRAVITY);

/**
 * Get absolute altitude from pressure (using simplified equation).
 * Referrence pressure is standard pressure at sea level
 *
 * @param pressure current pressure in Pascal (Pa)
 * @return altitude in pressure in ISA conditions
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
 *
 * @param pressure current pressure in Pascal (Pa)
 * @param ref reference pressure (QFE) when height = 0
 * @return altitude in pressure in ISA conditions
 */
static inline float pprz_isa_height_of_pressure(float pressure, float ref)
{
  if (pressure > 0. && ref > 0.) {
    return (PPRZ_ISA_M_OF_P_CONST * logf(ref / pressure));
  } else {
    return 0.;
  }
}

/**
 * Get pressure in Pa from absolute altitude (using simplified equation).
 *
 * @param altitude current absolute altitude in meters
 * @return static pressure in Pa in ISA conditions
 */
static inline float pprz_isa_pressure_of_altitude(float altitude)
{
  return (PPRZ_ISA_SEA_LEVEL_PRESSURE * expf((-1. / PPRZ_ISA_M_OF_P_CONST) * altitude));
}

/**
 * Get pressure in Pa from height (using simplified equation).
 *
 * @param altitude current relative altitude in meters
 * @param ref reference pressure (QFE) when height = 0
 * @return static pressure in Pa in ISA conditions
 */
static inline float pprz_isa_pressure_of_height(float altitude, float ref)
{
  return (ref * expf((-1. / PPRZ_ISA_M_OF_P_CONST) * altitude));
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_ISA_H */
