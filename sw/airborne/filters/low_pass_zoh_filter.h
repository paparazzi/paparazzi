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

#ifndef LOW_PASS_ZOH_FILTER_H
#define LOW_PASS_ZOH_FILTER_H

#include "std.h"
#include "math/pprz_algebra_int.h"


/**
 * @brief Zero Order Hold (ZOH) discrete first order low pass filter structure.
 *
 * This structure represents a first order low pass filter implemented using
 * the Zero Order Hold (ZOH) method for discretization. It maintains the necessary
 * state for filtering operations.
 */
struct FirstOrderZOHLowPass {
  float discrete_time_constant;
  float last_in;
  float last_out;
};

/**
 * @brief Init first order ZOH low pass filter.
 *
 * The ZOH discretization method is used to convert a continuous-time first order
 * low pass filter into its discrete-time equivalent which matches the continuous-time
 * behavior at the sampling instants exactly when holding the input constant between samples.
 *
 * @param[out] filter first order ZOH low pass filter structure
 * @param[in] tau time constant of the first order low pass filter [s]
 * @param[in] sample_time sampling period of the signal [s]
 * @param[in] value initial value of the filter
 */
static inline void init_first_order_zoh_low_pass(struct FirstOrderZOHLowPass *filter, const float tau,
    const float sample_time,
    float value)
{
  filter->discrete_time_constant = exp(-sample_time / tau);
  filter->last_in = value;
  filter->last_out = value;
}

/**
 * @brief Update first order ZOH low pass filter state with a new value.
 *
 * @param[in,out] filter first order ZOH low pass filter structure
 * @param[in] value new input value of the filter
 * @return new filtered value
 */
static inline float update_first_order_zoh_low_pass(struct FirstOrderZOHLowPass *filter, const float value)
{
  float out = (1.0f - filter->discrete_time_constant) * value + filter->discrete_time_constant * filter->last_out;
  filter->last_in = value;
  filter->last_out = out;
  return out;
}

/**
 * @brief Reset the first order ZOH low-pass filter to a specific value.
 *
 * @param[in,out] filter first order ZOH low pass filter structure
 * @param[in] value Value to reset the filter to
 * @return The reset value
 */
static inline float reset_first_order_zoh_low_pass(struct FirstOrderZOHLowPass *filter, const float value)
{
  filter->last_in = value;
  filter->last_out = value;
  return value;
}

/**
 * @brief Get current value of the first order ZOH low pass filter.
 *
 * @param[in] filter first order ZOH low pass filter structure
 * @return current value of the filter
 */
static inline float get_first_order_zoh_low_pass(const struct FirstOrderZOHLowPass *filter)
{
  return filter->last_out;
}

/** @brief Update time constant (tau parameter) for first order ZOH low pass filter
 * @param[in,out] filter first order ZOH low pass filter structure
 * @param[in] tau time constant of the  first order low pass filter [s]
 * @param[in] sample_time sampling period of the signal [s]
 */
static inline void update_first_order_zoh_low_pass_tau(struct FirstOrderZOHLowPass *filter, const float tau,
    const float sample_time)
{
  filter->discrete_time_constant = exp(-sample_time / tau);
}

#endif // LOW_PASS_ZOH_FILTER_H