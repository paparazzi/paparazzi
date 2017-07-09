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

extern int32_t mean_i(int32_t *array, uint32_t n_elements);
extern int32_t variance_i(int32_t *array, uint32_t n_elements);
extern int32_t covariance_i(int32_t *array1, int32_t *array2, uint32_t n_elements);

extern float mean_f(float *arr, uint32_t n_elements);
extern float variance_f(float *array, uint32_t n_elements);
extern float covariance_f(float *array1, float *array2, uint32_t n_elements);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_STAT_H */
