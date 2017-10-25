/*
 * Copyright (C) 2012 Gautier Hattenberger (ENAC), 2017 Kirk Scheper
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
 * @brief Statistics functions.
 *
 */

#ifndef PPRZ_STAT_H
#define PPRZ_STAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"

/** Compute the mean value of an array
 *  This is implemented using floats to handle scaling of all variables
 *  @param[in] *array The array
 *  @param[in] n_elements Number of elements in the array
 *  @return mean
 */
extern int32_t mean_i(int32_t *array, uint32_t n_elements);

/** Compute the variance of an array of values (integer).
 *  The variance is a measure of how far a set of numbers is spread out
 *  V(X) = E[(X-E[X])^2] = E[X^2] - E[X]^2
 *  where E[X] is the expected value of X
 *  This is implemented using floats to handle scaling of all variables
 *  @param array pointer to an array of integer
 *  @param n_elements number of elements in the array
 *  @return variance
 */
extern int32_t variance_i(int32_t *array, uint32_t n_elements);

/** Compute the covariance of two arrays
 *  V(X) = E[(X-E[X])(Y-E[Y])] = E[XY] - E[X]E[Y]
 *  where E[X] is the expected value of X
 *  This is implemented using floats to handle scaling of all variables
 *  @param[in] *array1 The first array
 *  @param[in] *array2 The second array
 *  @param[in] n_elements Number of elements in the arrays
 *  @return covariance
 */
extern int32_t covariance_i(int32_t *array1, int32_t *array2, uint32_t n_elements);


/** Compute the sum array elements
 *  @param[in] *array The first array
 *  @param[in] n_elements Number of elements in the arrays
 *  @return array sum
 */
extern float sum_f(float *array, uint32_t n_elements);

/** Compute the mean value of an array (float)
 *  @param[in] *array The array
 *  @param[in] n_elements Number of elements in the array
 *  @return mean
 */
extern float mean_f(float *arr, uint32_t n_elements);

/** Compute the variance of an array of values (float).
 *  The variance is a measure of how far a set of numbers is spread out
 *  V(X) = E[(X-E[X])^2] = E[X^2] - E[X]^2
 *  where E[X] is the expected value of X
 *  @param array Pointer to an array of float
 *  @param n_elements Number of values in the array
 *  @return variance
 */
extern float variance_f(float *array, uint32_t n_elements);

/** Compute the covariance of two arrays
 *  V(X) = E[(X-E[X])(Y-E[Y])] = E[XY] - E[X]E[Y]
 *  where E[X] is the expected value of X
 *  @param[in] *array1 The first array
 *  @param[in] *array2 The second array
 *  @param[in] n_elements Number of elements in the arrays
 *  @return covariance
 */
extern float covariance_f(float *array1, float *array2, uint32_t n_elements);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_STAT_H */
