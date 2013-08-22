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
 *
 */

/**
 * @file math/pprz_isa.h
 * @brief Paparazzi atmospheric pressure convertion utilities
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

#include "std.h"
#include <math.h>

// Standard Atmosphere constants
#define PPRZ_ISA_SEA_LEVEL_PRESSURE 101325.0
#define PPRZ_ISA_SEA_LEVEL_TEMP 288.15
#define PPRZ_ISA_TEMP_LAPS_RATE 0.0065
#define PPRZ_ISA_GRAVITY 9.80665
#define PPRZ_ISA_AIR_GAS_CONSTANT (8.31447/0.0289644)

#define PPRZ_ISA_M_OF_P_CONST (PPRZ_ISA_AIR_GAS_CONSTANT*PPRZ_ISA_SEA_LEVEL_TEMP/PPRZ_ISA_GRAVITY)

/**
 * Get meters from pressure (using simplified equation)
 *
 * @param pressure pressure (float) in Pascal (Pa)
 * @return altitude in pressure in ISA conditions
 */
static inline float pprz_isa_meters_of_pressure_f(float pressure) {
  return (PPRZ_ISA_M_OF_P_CONST*logf(PPRZ_ISA_SEA_LEVEL_PRESSURE/pressure));
}

/**
 * Get meters from pressure (using simplified equation)
 *
 * @param pressure pressure (int) in deci Pascal (10*Pa)
 * @return altitude in pressure in ISA conditions
 */
static inline float pprz_isa_meters_of_pressure_i(int32_t pressure) {
  float p = (float)pressure/10.;
  return pprz_isa_meters_of_pressure_f(p);
}


#endif /* PPRZ_ISA_H */
