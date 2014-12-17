/*
 * Copyright (C) 2012 Gautier Hattenberger (ENAC)
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
 * @file pprz_stat.h
 * @brief Statistics functions like variance.
 *
 */

#ifndef PPRZ_STAT_H
#define PPRZ_STAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"

/** Compute the variance of an array of values (float).
 *  The variance is a measure of how far a set of numbers is spread out
 *  V(X) = E[(X-E[X])^2] = E[X^2] - E[X]^2
 *  where E[X] is the expected value of X
 *
 *  @param array pointer to an array of float
 *  @param nb numbre of values in the array, must be >0
 *  @return variance
 */
static inline float variance_float(float *array, int nb)
{
  float me = 0.;
  float see = 0.;
  for (int i = 0; i < nb; i++) {
    me += array[i];
    see += array[i] * array[i];
  }
  me /= nb;
  return (see / nb - me * me);
}

/** Compute the variance of an array of values (integer).
 *  The variance is a measure of how far a set of numbers is spread out
 *  V(X) = E[(X-E[X])^2] = E[X^2] - E[X]^2
 *  where E[X] is the expected value of X
 *
 *  @param array pointer to an array of integer
 *  @param nb numbre of values in the array, must be >0
 *  @return variance
 */
static inline int32_t variance_int(int32_t *array, int nb)
{
  float me = 0;
  float see = 0;
  for (int i = 0; i < nb; i++) {
    me += (float)array[i];
    see += (float)(array[i] * array[i]);
  }
  me /= nb;
  return (see / nb - me * me);
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_STAT_H */
