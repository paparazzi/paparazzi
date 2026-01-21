/*
 *
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
 *
 */

/** @file filters/complementary_filter.h
 *  @brief Implementation of complementary filters (first order, second order, and Butterworth variants)
 *
 * Provides structures and functions to initialize, update, reset, and get outputs from
 * first order and second order complementary filters, as well as Butterworth complementary filters.
 *
 * FIXME: This implementation of complementary filters forces user to first integrate or differentiate
 * the signals outside of the filter before passing them in. Consider extending the filter to handle integration
 * and differentiation internally. Maybe implement cascaded complementary filters for this purpose.
 *
 * @author Justin Dubois <j.p.g.dubois@student.tudelft.nl>
 */

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "std.h"
#include "filters/low_pass_filter.h"


/**
 * @brief First order complementary filter structure.
 *
 * This structure contains two first order low-pass filters,
 * one for the high-pass path and one for the low-pass path.
 */
struct FirstOrderComplementary {
  struct FirstOrderLowPass x_lp_filter; // Low pass filter instance for high pass path
  struct FirstOrderLowPass y_lp_filter; // Low pass filter instance for low pass path
};

/** Initialize the first order complementary filter.
 *
 * @param filter Complementary filter struct
 * @param cut_off Time constant of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_first_order_complementary(
  struct FirstOrderComplementary *filter,
  float cut_off, float sample_time,
  float value)
{
  init_first_order_low_pass(&filter->x_lp_filter, 1.0f / cut_off, sample_time, value);
  init_first_order_low_pass(&filter->y_lp_filter, 1.0f / cut_off, sample_time, value);
}

/** Update the first order complementary filter with new input values.
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_first_order_complementary(
  struct FirstOrderComplementary *filter,
  float value_x, float value_y)
{
  float x_lp_output = update_first_order_low_pass(&filter->x_lp_filter, value_x);
  float y_lp_output = update_first_order_low_pass(&filter->y_lp_filter, value_y);
  return filter->x_lp_filter.last_in - x_lp_output + y_lp_output;
}

/**
 * @brief Reset the first order complementary filter to a specific value.
 *
 * @param filter Complementary filter struct
 * @param value Value to reset the filter to
 * @return The reset value
 */
static inline float reset_first_order_complementary(
  struct FirstOrderComplementary *filter,
  float value)
{
  reset_first_order_low_pass(&filter->x_lp_filter, value);
  reset_first_order_low_pass(&filter->y_lp_filter, value);
  return value;
}

/** Get current value of the first order complementary filter.
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_first_order_complementary(const struct FirstOrderComplementary *filter)
{
  float x_lp_output = get_first_order_low_pass(&filter->x_lp_filter);
  float y_lp_output = get_first_order_low_pass(&filter->y_lp_filter);
  return filter->x_lp_filter.last_in - x_lp_output + y_lp_output;
}

/**
 * @brief Second order complementary filter structure.
 *
 * This structure contains two second order low-pass filters,
 * one for the high-pass path and one for the low-pass path.
 */
struct SecondOrderComplementary {
  struct SecondOrderLowPass x_lp_filter; // Low pass filter instance for high pass path
  struct SecondOrderLowPass y_lp_filter; // Low pass filter instance for low pass path
};

/** Initialize the second order complementary filter.
 *
 * @param filter Complementary filter struct
 * @param cut_off Time constant of the low-pass filter
 * @param Q Q factor of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_second_order_complementary(
  struct SecondOrderComplementary *filter,
  float cut_off, float Q, float sample_time,
  float value)
{
  init_second_order_low_pass(&filter->x_lp_filter, 1.0f / cut_off, Q, sample_time, value);
  init_second_order_low_pass(&filter->y_lp_filter, 1.0f / cut_off, Q, sample_time, value);
}

/** Update the second order complementary filter with new input values.
 *
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_second_order_complementary(
  struct SecondOrderComplementary *filter,
  float value_x, float value_y)
{
  float x_lp_output = update_second_order_low_pass(&filter->x_lp_filter, value_x);
  float y_lp_output = update_second_order_low_pass(&filter->y_lp_filter, value_y);
  return filter->x_lp_filter.i[0] - x_lp_output + y_lp_output;
}

/**
 * @brief Reset the second order complementary filter to a specific value.
 *
 * @param filter Complementary filter struct
 * @param value Value to reset the filter to
 * @return The reset value
 */
static inline float reset_second_order_complementary(
  struct SecondOrderComplementary *filter,
  float value)
{
  reset_second_order_low_pass(&filter->x_lp_filter, value);
  reset_second_order_low_pass(&filter->y_lp_filter, value);
  return value;
}

/** Get current value of the second order complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_second_order_complementary(const struct SecondOrderComplementary *filter)
{
  float x_lp_output = get_second_order_low_pass(&filter->x_lp_filter);
  float y_lp_output = get_second_order_low_pass(&filter->y_lp_filter);
  return filter->x_lp_filter.i[0] - x_lp_output + y_lp_output;
}

typedef struct SecondOrderComplementary Butterworth2Complementary;

/** Initialize the Butterworth 2nd order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @param cut_off Time constant of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_butterworth_2_complementary(
  Butterworth2Complementary *filter,
  float cut_off, float sample_time,
  float value)
{
  init_second_order_complementary(
    (struct SecondOrderComplementary *)filter,
    cut_off, 0.7071, sample_time,
    value);
}

/** Update the Butterworth 2nd order low-pass complementary filter with new input values.
 *
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_butterworth_2_complementary(Butterworth2Complementary *filter, float value_x, float value_y)
{
  return update_second_order_complementary((struct SecondOrderComplementary *)filter, value_x, value_y);
}

/**
 * @brief Reset the Butterworth 2nd order complementary filter to a specific value.
 *
 * @param filter Complementary filter struct
 * @param value Value to reset the filter to
 */
static inline void reset_butterworth_2_complementary(Butterworth2Complementary *filter, float value)
{
  reset_second_order_complementary((struct SecondOrderComplementary *)filter, value);
}

/** Get current value of the Butterworth 2nd order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_butterworth_2_complementary(const Butterworth2Complementary *filter)
{
  return get_second_order_complementary((const struct SecondOrderComplementary *)filter);
}

/**
 * @brief 4th order Butterworth complementary filter structure.
 *
 * This structure contains two 4th order Butterworth low-pass filters,
 * one for the high-pass path and one for the low-pass path.
 */
typedef struct {
  Butterworth4LowPass x_lp_filter; // Low pass filter instance for high pass path
  Butterworth4LowPass y_lp_filter; // Low pass filter instance for low pass path
} Butterworth4Complementary;

/** Initialize the Butterworth 4th order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @param cut_off Time constant of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_butterworth_4_complementary(Butterworth4Complementary *filter, float cut_off, float sample_time,
    float value)
{
  init_butterworth_4_low_pass(&filter->x_lp_filter, 1.0f / cut_off, sample_time, value);
  init_butterworth_4_low_pass(&filter->y_lp_filter, 1.0f / cut_off, sample_time, value);
}

/** Update the Butterworth 4th order low-pass complementary filter with new input values.
 *
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_butterworth_4_complementary(Butterworth4Complementary *filter, float value_x, float value_y)
{
  float x_lp_output = update_butterworth_4_low_pass(&filter->x_lp_filter, value_x);
  float y_lp_output = update_butterworth_4_low_pass(&filter->y_lp_filter, value_y);
  return filter->x_lp_filter.lp1.i[0] - x_lp_output + y_lp_output;
}

/**
 * @brief Reset the Butterworth 4th order complementary filter to a specific value.
 *
 * @param filter Complementary filter struct
 * @param value Value to reset the filter to
 */
static inline void reset_butterworth_4_complementary(Butterworth4Complementary *filter, float value)
{
  reset_butterworth_4_low_pass(&filter->x_lp_filter, value);
  reset_butterworth_4_low_pass(&filter->y_lp_filter, value);
}

/** Get current value of the Butterworth 4th order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_butterworth_4_complementary(const Butterworth4Complementary *filter)
{
  float x_lp_output = get_butterworth_4_low_pass(&filter->x_lp_filter);
  float y_lp_output = get_butterworth_4_low_pass(&filter->y_lp_filter);
  return filter->x_lp_filter.lp1.i[0] - x_lp_output + y_lp_output;
}








#endif /* COMPLEMENTARY_FILTER_H */