/*
 * Copyright (C) 2011  Felix Ruess <felix.ruess@gmail.com>
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
 * @file pprz_geodetic_utm.h
 * @brief Constants UTM (Mercator) projections.
 * @addtogroup math_geodetic
 * @{
 * @addtogroup math_geodetic_utm UTM (Mercator) projections
 * @{
 */

#ifndef PPRZ_GEODETIC_UTM_H
#define PPRZ_GEODETIC_UTM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"

/* Computation for the WGS84 geoid only */
#define E 0.08181919106
#define K0 0.9996
#define DELTA_EAST  500000.
#define DELTA_NORTH 0.
#define A 6378137.0
#define N (K0*A)

#define LambdaOfUtmZone(utm_zone) RadOfDeg((utm_zone-1)*6-180+3)
#define UtmZoneOfLlaLonRad(lla_lon) (floor(DegOfRad(lla_lon) + 180) / 6) + 1
#define UtmZoneOfLlaLonDeg(lla_lon) (floor((lla_lon) / 1e7 + 180) / 6) + 1

static const float serie_coeff_proj_mercator[5] = {
  0.99832429842242842444,
  0.00083632803657738403,
  0.00000075957783563707,
  0.00000000119563131778,
  0.00000000000241079916
};

static const float serie_coeff_proj_mercator_inverse[5] = {
  0.998324298422428424,
  0.000837732168742475825,
  5.90586914811817062e-08,
  1.6734091890305064e-10,
  2.13883575853313883e-13
};

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_GEODETIC_UTM_H */
/** @}*/
/** @}*/
