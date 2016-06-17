/*
 * Copyright (C) 2010  Christophe De Wagter
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
 * @file pprz_geodetic_wgs84.h
 * @brief WGS-84 Geoid Heights.
 *
 * Ten by Ten Degree WGS-84 Geoid Heights from -180 to +170 Degrees of Longitude
 *
 * Geoid height approximations in meters
 *
 * Source:
 * Defense Mapping Agency. 12 Jan 1987. GPS UE Relevant WGS-84 Data Base Package. Washington, DC: Defense Mapping Agency
 *
 * Link:
 * http://www.colorado.edu/geography/gcraft/notes/datum/geoid84.html
 *
 * rows are from -180 to +170 starting north +90 to south-90
 *
 * @addtogroup math_geodetic
 * @{
 * @addtogroup math_geodetic_wgs84 WGS-84 Geoid
 * @{
 */

#ifndef PPRZ_GEODETIC_WGS84_H
#define PPRZ_GEODETIC_WGS84_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"

static const int8_t pprz_geodetic_wgs84_int[19][36] = {
  {13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
  {3, 1, -2, -3, -3, -3, -1, 3, 1, 5, 9, 11, 19, 27, 31, 34, 33, 34, 33, 34, 28, 23, 17, 13, 9, 4, 4, 1, -2, -2, 0, 2, 3, 2, 1, 1},
  {2, 2, 1, -1, -3, -7, -14, -24, -27, -25, -19, 3, 24, 37, 47, 60, 61, 58, 51, 43, 29, 20, 12, 5, -2, -10, -14, -12, -10, -14, -12, -6, -2, 3, 6, 4},
  {2, 9, 17, 10, 13, 1, -14, -30, -39, -46, -42, -21, 6, 29, 49, 65, 60, 57, 47, 41, 21, 18, 14, 7, -3, -22, -29, -32, -32, -26, -15, -2, 13, 17, 19, 6},
  { -8, 8, 8, 1, -11, -19, -16, -18, -22, -35, -40, -26, -12, 24, 45, 63, 62, 59, 47, 48, 42, 28, 12, -10, -19, -33, -43, -42, -43, -29, -2, 17, 23, 22, 6, 2},
  { -12, -10, -13, -20, -31, -34, -21, -16, -26, -34, -33, -35, -26, 2, 33, 59, 52, 51, 52, 48, 35, 40, 33, -9, -28, -39, -48, -59, -50, -28, 3, 23, 37, 18, -1, -11},
  { -7, -5, -8, -15, -28, -40, -42, -29, -22, -26, -32, -51, -40, -17, 17, 31, 34, 44, 36, 28, 29, 17, 12, -20, -15, -40, -33, -34, -34, -28, 7, 29, 43, 20, 4, -6},
  {5, 10, 7, -7, -23, -39, -47, -34, -9, -10, -20, -45, -48, -32, -9, 17, 25, 31, 31, 26, 15, 6, 1, -29, -44, -61, -67, -59, -36, -11, 21, 39, 49, 39, 22, 10},
  {13, 12, 11, 2, -11, -28, -38, -29, -10, 3, 1, -11, -41, -42, -16, 3, 17, 33, 22, 23, 2, -3, -7, -36, -59, -90, -95, -63, -24, 12, 53, 60, 58, 46, 36, 26},
  {22, 16, 17, 13, 1, -12, -23, -20, -14, -3, 14, 10, -15, -27, -18, 3, 12, 20, 18, 12, -13, -9, -28, -49, -62, -89, -102, -63, -9, 33, 58, 73, 74, 63, 50, 32},
  {36, 22, 11, 6, -1, -8, -10, -8, -11, -9, 1, 32, 4, -18, -13, -9, 4, 14, 12, 13, -2, -14, -25, -32, -38, -60, -75, -63, -26, 0, 35, 52, 68, 76, 64, 52},
  {51, 27, 10, 0, -9, -11, -5, -2, -3, -1, 9, 35, 20, -5, -6, -5, 0, 13, 17, 23, 21, 8, -9, -10, -11, -20, -40, -47, -45, -25, 5, 23, 45, 58, 57, 63},
  {46, 22, 5, -2, -8, -13, -10, -7, -4, 1, 9, 32, 16, 4, -8, 4, 12, 15, 22, 27, 34, 29, 14, 15, 15, 7, -9, -25, -37, -39, -23, -14, 15, 33, 34, 45},
  {21, 6, 1, -7, -12, -12, -12, -10, -7, -1, 8, 23, 15, -2, -6, 6, 21, 24, 18, 26, 31, 33, 39, 41, 30, 24, 13, -2, -20, -32, -33, -27, -14, -2, 5, 20},
  { -15, -18, -18, -16, -17, -15, -10, -10, -8, -2, 6, 14, 13, 3, 3, 10, 20, 27, 25, 26, 34, 39, 45, 45, 38, 39, 28, 13, -1, -15, -22, -22, -18, -15, -14, -10},
  { -45, -43, -37, -32, -30, -26, -23, -22, -16, -10, -2, 10, 20, 20, 21, 24, 22, 17, 16, 19, 25, 30, 35, 35, 33, 30, 27, 10, -2, -14, -23, -30, -33, -29, -35, -43},
  { -61, -60, -61, -55, -49, -44, -38, -31, -25, -16, -6, 1, 4, 5, 4, 2, 6, 12, 16, 16, 17, 21, 20, 26, 26, 22, 16, 10, -1, -16, -29, -36, -46, -55, -54, -59},
  { -53, -54, -55, -52, -48, -42, -38, -38, -29, -26, -26, -24, -23, -21, -19, -16, -12, -8, -4, -1, 1, 4, 4, 6, 5, 4, 2, -6, -15, -24, -33, -40, -48, -50, -53, -52},
  { -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30}
};

#define WGS84_H(x,y) ((float) pprz_geodetic_wgs84_int[(y)][(x)])

/** Get WGS84 ellipsoid/geoid separation.
 * @param[in] lat Latitude in 1e7deg
 * @param[in] lon Longitude in 1e7deg
 * @return geoid separation in m
 */
static inline float wgs84_ellipsoid_to_geoid_i(int32_t lat, int32_t lon)
{
  float x = (180.0f + (float)lon / 1e7) / 10.0f;
  Bound(x, 0.0f, 35.99999f);
  float y = (90.0f - (float)lat / 1e7) / 10.0f;
  Bound(y, 0.0f, 17.99999f);
  uint8_t ex1 = (uint8_t) x;
  uint8_t ex2 = ex1 + 1;
  if (ex2 >= 36) { ex2 = 0; }
  uint8_t ey1 = (uint8_t) y;
  uint8_t ey2 = ey1 + 1;
  float lin_x = x - ((float) ex1);
  float lin_y = y - ((float) ey1);
  float h11 = (1.0f - lin_x) * (1.0f - lin_y) * WGS84_H(ex1, ey1);
  float h12 = lin_x * (1.0f - lin_y) * WGS84_H(ex2, ey1);
  float h21 = (1.0f - lin_x) * lin_y * WGS84_H(ex1, ey2);
  float h22 = lin_x * lin_y * WGS84_H(ex2, ey2);
  return h11 + h12 + h21 + h22;
}

static inline float wgs84_ellipsoid_to_geoid(float lat, float lon)
{
  float x = (180.0f + DegOfRad(lon)) / 10.0f;
  Bound(x, 0.0f, 35.99999f);
  float y = (90.0f - DegOfRad(lat)) / 10.0f;
  Bound(y, 0.0f, 17.99999f);
  uint8_t ex1 = (uint8_t) x;
  uint8_t ex2 = ex1 + 1;
  if (ex2 >= 36) { ex2 = 0; }
  uint8_t ey1 = (uint8_t) y;
  uint8_t ey2 = ey1 + 1;
  float lin_x = x - ((float) ex1);
  float lin_y = y - ((float) ey1);
  float h11 = (1.0f - lin_x) * (1.0f - lin_y) * WGS84_H(ex1, ey1);
  float h12 = lin_x * (1.0f - lin_y) * WGS84_H(ex2, ey1);
  float h21 = (1.0f - lin_x) * lin_y * WGS84_H(ex1, ey2);
  float h22 = lin_x * lin_y * WGS84_H(ex2, ey2);
  return h11 + h12 + h21 + h22;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_GEODETIC_WGS84_H */
/** @}*/
/** @}*/
