/*
 * Copyright (C) 2025 Justin Dubois <j.p.g.dubois@student.tudelft.nl>
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
/** @file low_pass_zoh_filter.h
 *  @brief A first order low pass filter using Zero Order Hold (ZOH) discretization.
 */
/** @file filters/low_pass_zoh_filter.h
 *  @brief Definitions and inline functions for 1st order low-pass ZOH filter vector types
 *
 * Provides structures and functions to initialize, update, reset, and get outputs from
 * 1st order low-pass ZOH filter vectors.
 *
 * @author Justin Dubois <j.p.g.dubois@student.tudelft.nl>
 */

#ifndef LOW_PASS_ZOH_FILTER_TYPES_H
#define LOW_PASS_ZOH_FILTER_TYPES_H

#include "std.h"
#include "filters/low_pass_zoh_filter.h"
#include "math/pprz_algebra_float.h"

/**
 * @brief Initialize a set of first order ZOH low-pass filters to zero for 3D vector data.
 * @param[out] filter_array Array of FirstOrderZOHLowPass filters to initialize.
 * @param[in] omega Cut-off frequencies for the filters (rad/s).
 * @param[in] sample_time Sampling time interval (seconds).
 */
static inline void init_first_order_zoh_low_pass_array(const uint8_t n,
    struct FirstOrderZOHLowPass filter_array[restrict n], const float omega[restrict n], const float sample_time)
{
  for (uint8_t i = 0; i < n; i++) {
    float tau = 1.0f / omega[i];
    init_first_order_zoh_low_pass(&filter_array[i], tau, sample_time, 0.0f);
  }
}

/**
 * @brief Update an array of first order ZOH low-pass filters.
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of FirstOrderZOHLowPass filters to update.
 * @param[in] input_array Array of input values for the filters.
 */
static inline void update_first_order_zoh_low_pass_array(const uint8_t n,
    struct FirstOrderZOHLowPass filter_array[restrict n], const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    update_first_order_zoh_low_pass(&filter_array[i], input_array[i]);
  }
}

/**
 * @brief Reset an array of first order ZOH low-pass filters to specific values.
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of FirstOrderZOHLowPass filters to reset.
 * @param[in] value_array Array of values to reset the filters to.
 */
static inline void reset_first_order_zoh_low_pass_array(const uint8_t n,
    struct FirstOrderZOHLowPass filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    filter_array[i].last_in = value_array[i];
    filter_array[i].last_out = value_array[i];
  }
}

/**
 * @brief Retrieve the filtered outputs from an array of first order ZOH low-pass filters.
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of FirstOrderZOHLowPass filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_first_order_zoh_low_pass_array(const uint8_t n,
    const struct FirstOrderZOHLowPass filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    output_array[i] = filter_array[i].last_out;
  }
}

#endif // LOW_PASS_ZOH_FILTER_TYPES_H