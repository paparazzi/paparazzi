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

/** @file filters/complementary_filter_types.h
 *  @brief Definitions and inline functions for 1st order complementary filter vector types
 *
 *  Provides structures and functions to initialize, update, reset, and get outputs from
 * 1st order complementary filter vectors.
 *
 * @author Justin Dubois <j.p.g.dubois@student.tudelft.nl>
 */

#ifndef COMPLEMENTARY_FILTER_TYPES_H
#define COMPLEMENTARY_FILTER_TYPES_H

#include "std.h"
#include "filters/complementary_filter.h"
#include "math/pprz_algebra_float.h"

/**
 * @brief 3D vector of 1st order complementary filters.
 *
 * This structure contains three 1st order complementary filters
 * for the x, y, and z components.
 */
struct FirstOrderComplementaryVect3 {
  struct FirstOrderComplementary x;
  struct FirstOrderComplementary y;
  struct FirstOrderComplementary z;
};

/**
 * @brief Initialize 3D vector of 1st order complementary filters.
 *
 * @param[out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_first_order_complementary_vect3(struct FirstOrderComplementaryVect3 *filter,
    const struct FloatVect3 *cut_off, float sample_time)
{
  init_first_order_complementary(&filter->x, cut_off->x, sample_time, 0.0f);
  init_first_order_complementary(&filter->y, cut_off->y, sample_time, 0.0f);
  init_first_order_complementary(&filter->z, cut_off->z, sample_time, 0.0f);
}

/**
 * @brief Get current output 3D vector from 1st order complementary filters.
 *
 * @param[in] filter Pointer to FirstOrderComplementaryVect3 struct.
 * @return FloatVect3 containing filtered outputs.
 */
static inline struct FloatVect3 get_first_order_complementary_vect3(const struct FirstOrderComplementaryVect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_first_order_complementary(&filter->x);
  output.y = get_first_order_complementary(&filter->y);
  output.z = get_first_order_complementary(&filter->z);
  return output;
}

/**
 * @brief Get current output rates from 1st order complementary filters.
 *
 * @param[in] filter Pointer to FirstOrderComplementaryVect3 struct.
 * @return FloatRates containing filtered outputs p, q, r.
 */
static inline struct FloatRates get_first_order_complementary_rates(const struct FirstOrderComplementaryVect3 *filter)
{
  struct FloatRates output;
  output.p = get_first_order_complementary(&filter->x);
  output.q = get_first_order_complementary(&filter->y);
  output.r = get_first_order_complementary(&filter->z);
  return output;
}

/**
 * @brief Update 3D vector of 1st order complementary filters.
 *
 * @param[in,out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatVect3 input vector x values.
 * @param[in] value_y Pointer to FloatVect3 input vector y values.
 */
static inline struct FloatVect3 update_first_order_complementary_vect3(struct FirstOrderComplementaryVect3 *filter,
    const struct FloatVect3 *value_x, const struct FloatVect3 *value_y)
{
  update_first_order_complementary(&filter->x, value_x->x, value_y->x);
  update_first_order_complementary(&filter->y, value_x->y, value_y->y);
  update_first_order_complementary(&filter->z, value_x->z, value_y->z);
  return get_first_order_complementary_vect3(filter);
}

/**
 * @brief Update 3D vector of 1st order complementary filters for rates.
 *
 * @param[in,out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatRates input p, q, r values.
 * @param[in] value_y Pointer to FloatRates input p, q, r values.
 */
static inline struct FloatRates update_first_order_complementary_rates(struct FirstOrderComplementaryVect3 *filter,
    const struct FloatRates *value_x, const struct FloatRates *value_y)
{
  update_first_order_complementary(&filter->x, value_x->p, value_y->p);
  update_first_order_complementary(&filter->y, value_x->q, value_y->q);
  update_first_order_complementary(&filter->z, value_x->r, value_y->r);
  return get_first_order_complementary_rates(filter);
}

/**
 * @brief Reset 3D vector of 1st order complementary filters to a specific value.
 *
 * @param[in,out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] value Value to reset the filters to.
 */
static inline void reset_first_order_complementary_vect3(struct FirstOrderComplementaryVect3 *filter,
    const struct FloatVect3 *value)
{
  reset_first_order_complementary(&filter->x, value->x);
  reset_first_order_complementary(&filter->y, value->y);
  reset_first_order_complementary(&filter->z, value->z);
}

/**
 * @brief Reset 3D vector of 1st order complementary filters for rates to a specific value.
 *
 * @param[in,out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] value Value to reset the filters to.
 */
static inline void reset_first_order_complementary_rates(struct FirstOrderComplementaryVect3 *filter,
    const struct FloatRates *value)
{
  reset_first_order_complementary(&filter->x, value->p);
  reset_first_order_complementary(&filter->y, value->q);
  reset_first_order_complementary(&filter->z, value->r);
}

/**
 * @brief Initialize an array of 1st order complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of FirstOrderComplementary filters.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_first_order_complementary_array(uint8_t n,
    struct FirstOrderComplementary filter_array[restrict n], const float cut_off[restrict n], float sample_time)
{
  for (uint8_t i = 0; i < n; i++) {
    init_first_order_complementary(&filter_array[i], cut_off[i], sample_time, 0.0f);
  }
}

/**
 * @brief Update an array of 1st order complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of FirstOrderComplementary filters.
 * @param[in] value_x_array Array of input x values.
 * @param[in] value_y_array Array of input y values.
 */
static inline void update_first_order_complementary_array(uint8_t n,
    struct FirstOrderComplementary filter_array[restrict n], const float value_x_array[restrict n],
    const float value_y_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    update_first_order_complementary(&filter_array[i], value_x_array[i], value_y_array[i]);
  }
}

/**
 * @brief Reset an array of 1st order complementary filters to specific values.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of FirstOrderComplementary filters.
 * @param[in] value_array Array of values to reset the filters to.
 */
static inline void reset_first_order_complementary_array(uint8_t n,
    struct FirstOrderComplementary filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    reset_first_order_complementary(&filter_array[i], value_array[i]);
  }
}

/**
 * @brief Get current outputs from an array of 1st order complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of FirstOrderComplementary filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_first_order_complementary_array(uint8_t n,
    const struct FirstOrderComplementary filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    output_array[i] = get_first_order_complementary(&filter_array[i]);
  }
}

/**
 * @brief 3D vector of 2nd order Butterworth complementary filters.
 *
 * This structure contains three 2nd order Butterworth complementary filters
 * for the x, y, and z components.
 */
struct Butterworth2ComplementaryVect3 {
  Butterworth2Complementary x;
  Butterworth2Complementary y;
  Butterworth2Complementary z;
};

/**
 * @brief Initialize 3D vector of 2nd order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_2_complementary_vect3(struct Butterworth2ComplementaryVect3 *filter,
    const struct FloatVect3 *cut_off, float sample_time)
{
  init_butterworth_2_complementary(&filter->x, cut_off->x, sample_time, 0.0f);
  init_butterworth_2_complementary(&filter->y, cut_off->y, sample_time, 0.0f);
  init_butterworth_2_complementary(&filter->z, cut_off->z, sample_time, 0.0f);
}

/**
 * @brief Get current output 3D vector from 2nd order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth2ComplementaryVect3 struct.
 * @return FloatVect3 containing filtered outputs.
 */
static inline struct FloatVect3 get_butterworth_2_complementary_vect3(const struct Butterworth2ComplementaryVect3
    *filter)
{
  struct FloatVect3 output;
  output.x = get_butterworth_2_complementary(&filter->x);
  output.y = get_butterworth_2_complementary(&filter->y);
  output.z = get_butterworth_2_complementary(&filter->z);
  return output;
}

/**
 * @brief Get current output rates from 2nd order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth2ComplementaryVect3 struct.
 * @return FloatRates containing filtered outputs p, q, r.
 */
static inline struct FloatRates get_butterworth_2_complementary_rates(const struct Butterworth2ComplementaryVect3
    *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_2_complementary(&filter->x);
  output.q = get_butterworth_2_complementary(&filter->y);
  output.r = get_butterworth_2_complementary(&filter->z);
  return output;
}

/**
 * @brief Initialize 3D vector of 2nd order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline struct FloatVect3 update_butterworth_2_complementary_vect3(struct Butterworth2ComplementaryVect3 *filter,
    const struct FloatVect3 *value_x, const struct FloatVect3 *value_y)
{
  update_butterworth_2_complementary(&filter->x, value_x->x, value_y->x);
  update_butterworth_2_complementary(&filter->y, value_x->y, value_y->y);
  update_butterworth_2_complementary(&filter->z, value_x->z, value_y->z);
  return get_butterworth_2_complementary_vect3(filter);
}

/**
 * @brief Update 3D vector of 2nd order Butterworth complementary filters.
 *
 * @param[in,out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatVect3 input vector x values.
 * @param[in] value_y Pointer to FloatVect3 input vector y values.
 */
static inline struct FloatRates update_butterworth_2_complementary_rates(struct Butterworth2ComplementaryVect3 *filter,
    const struct FloatRates *value_x, const struct FloatRates *value_y)
{
  update_butterworth_2_complementary(&filter->x, value_x->p, value_y->p);
  update_butterworth_2_complementary(&filter->y, value_x->q, value_y->q);
  update_butterworth_2_complementary(&filter->z, value_x->r, value_y->r);
  return get_butterworth_2_complementary_rates(filter);
}

/**
 * @brief Reset 3D vector of 2nd order Butterworth complementary filters to a specific value.
 *
 * @param[in,out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] value Value to reset the filters to.
 */
static inline void reset_butterworth_2_complementary_vect3(struct Butterworth2ComplementaryVect3 *filter,
    const struct FloatVect3 *value)
{
  reset_butterworth_2_complementary(&filter->x, value->x);
  reset_butterworth_2_complementary(&filter->y, value->y);
  reset_butterworth_2_complementary(&filter->z, value->z);
}

/**
 * @brief Reset 3D vector of 2nd order Butterworth complementary filters for rates to a specific value.
 *
 * @param[in,out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] value Value to reset the filters to.
 */
static inline void reset_butterworth_2_complementary_rates(struct Butterworth2ComplementaryVect3 *filter,
    const struct FloatRates *value)
{
  reset_butterworth_2_complementary(&filter->x, value->p);
  reset_butterworth_2_complementary(&filter->y, value->q);
  reset_butterworth_2_complementary(&filter->z, value->r);
}

/**
 * @brief Initialize an array of 2nd order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2Complementary filters.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_2_complementary_array(uint8_t n, Butterworth2Complementary filter_array[restrict n],
    const float cut_off[restrict n], float sample_time)
{
  for (uint8_t i = 0; i < n; i++) {
    init_butterworth_2_complementary(&filter_array[i], cut_off[i], sample_time, 0.0f);
  }
}

/**
 * @brief Update an array of 2nd order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth2Complementary filters.
 * @param[in] value_x_array Array of input x values.
 * @param[in] value_y_array Array of input y values.
 */
static inline void update_butterworth_2_complementary_array(uint8_t n,
    Butterworth2Complementary filter_array[restrict n], const float value_x_array[restrict n],
    const float value_y_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    update_butterworth_2_complementary(&filter_array[i], value_x_array[i], value_y_array[i]);
  }
}

/**
 * @brief Reset an array of 2nd order Butterworth complementary filters to specific values.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth2Complementary filters.
 * @param[in] value_array Array of values to reset the filters to.
 */
static inline void reset_butterworth_2_complementary_array(uint8_t n,
    Butterworth2Complementary filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    reset_butterworth_2_complementary(&filter_array[i], value_array[i]);
  }
}
/**
 * @brief Get current outputs from an array of 2nd order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth2Complementary filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_2_complementary_array(uint8_t n,
    const Butterworth2Complementary filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    output_array[i] = get_butterworth_2_complementary(&filter_array[i]);
  }
}

/**
 * @brief 3D vector of 4th order Butterworth complementary filters.
 *
 * This structure contains three 4th order Butterworth complementary filters
 * for the x, y, and z components.
 */
struct Butterworth4ComplementaryVect3 {
  Butterworth4Complementary x;
  Butterworth4Complementary y;
  Butterworth4Complementary z;
};

/**
 * @brief Initialize 3D vector of 4th order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_4_complementary_vect3(struct Butterworth4ComplementaryVect3 *filter,
    const struct FloatVect3 *cut_off, float sample_time)
{
  init_butterworth_4_complementary(&filter->x, cut_off->x, sample_time, 0.0f);
  init_butterworth_4_complementary(&filter->y, cut_off->y, sample_time, 0.0f);
  init_butterworth_4_complementary(&filter->z, cut_off->z, sample_time, 0.0f);
}

/**
 * @brief Get current output 3D vector from 4th order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth4ComplementaryVect3 struct.
 * @return FloatVect3 containing filtered outputs.
 */
static inline struct FloatVect3 get_butterworth_4_complementary_vect3(const struct Butterworth4ComplementaryVect3
    *filter)
{
  struct FloatVect3 output;
  output.x = get_butterworth_4_complementary(&filter->x);
  output.y = get_butterworth_4_complementary(&filter->y);
  output.z = get_butterworth_4_complementary(&filter->z);
  return output;
}

/**
 * @brief Get current output rates from 4th order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth4ComplementaryVect3 struct.
 * @return FloatRates containing filtered outputs p, q, r.
 */
static inline struct FloatRates get_butterworth_4_complementary_rates(const struct Butterworth4ComplementaryVect3
    *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_4_complementary(&filter->x);
  output.q = get_butterworth_4_complementary(&filter->y);
  output.r = get_butterworth_4_complementary(&filter->z);
  return output;
}

/**
 * @brief Initialize 3D vector of 4th order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline struct FloatVect3 update_butterworth_4_complementary_vect3(struct Butterworth4ComplementaryVect3 *filter,
    const struct FloatVect3 *value_x, const struct FloatVect3 *value_y)
{
  update_butterworth_4_complementary(&filter->x, value_x->x, value_y->x);
  update_butterworth_4_complementary(&filter->y, value_x->y, value_y->y);
  update_butterworth_4_complementary(&filter->z, value_x->z, value_y->z);
  return get_butterworth_4_complementary_vect3(filter);
}

/**
 * @brief Update 3D vector of 4th order Butterworth complementary filters for rates.
 *
 * @param[in,out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatRates input p, q, r values.
 * @param[in] value_y Pointer to FloatRates input p, q, r values.
 */
static inline struct FloatRates update_butterworth_4_complementary_rates(struct Butterworth4ComplementaryVect3 *filter,
    const struct FloatRates *value_x, const struct FloatRates *value_y)
{
  update_butterworth_4_complementary(&filter->x, value_x->p, value_y->p);
  update_butterworth_4_complementary(&filter->y, value_x->q, value_y->q);
  update_butterworth_4_complementary(&filter->z, value_x->r, value_y->r);
  return get_butterworth_4_complementary_rates(filter);
}

/**
 * @brief Reset 3D vector of 4th order Butterworth complementary filters to a specific value.
 *
 * @param[in,out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] value Value to reset the filters to.
 */
static inline void reset_butterworth_4_complementary_vect3(struct Butterworth4ComplementaryVect3 *filter,
    const struct FloatVect3 *value)
{
  reset_butterworth_4_complementary(&filter->x, value->x);
  reset_butterworth_4_complementary(&filter->y, value->y);
  reset_butterworth_4_complementary(&filter->z, value->z);
}

/**
 * @brief Reset 3D vector of 4th order Butterworth complementary filters for rates to a specific value.
 *
 * @param[in,out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] value Value to reset the filters to.
 */
static inline void reset_butterworth_4_complementary_rates(struct Butterworth4ComplementaryVect3 *filter,
    const struct FloatRates *value)
{
  reset_butterworth_4_complementary(&filter->x, value->p);
  reset_butterworth_4_complementary(&filter->y, value->q);
  reset_butterworth_4_complementary(&filter->z, value->r);
}

/**
 * @brief Initialize an array of 4th order Butterworth complementary filters.
 *
 * @param[in] n Number of filters to initialize.
 * @param[out] filter_array Array of Butterworth4Complementary filters.
 * @param[in] cut_off Time constant in seconds.
 * @param[in] sample_time Sampling period (seconds).
 */
static inline void init_butterworth_4_complementary_array(uint8_t n, Butterworth4Complementary filter_array[restrict n],
    const float cut_off[restrict n], float sample_time)
{
  for (uint8_t i = 0; i < n; i++) {
    init_butterworth_4_complementary(&filter_array[i], cut_off[i], sample_time, 0.0f);
  }
}

/**
 * @brief Update an array of 4th order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth4Complementary filters.
 * @param[in] value_x_array Array of input x values.
 * @param[in] value_y_array Array of input y values.
 */
static inline void update_butterworth_4_complementary_array(uint8_t n,
    Butterworth4Complementary filter_array[restrict n], const float value_x_array[restrict n],
    const float value_y_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    update_butterworth_4_complementary(&filter_array[i], value_x_array[i], value_y_array[i]);
  }
}

/**
 * @brief Reset an array of 4th order Butterworth complementary filters to specific values.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth4Complementary filters.
 * @param[in] value_array Array of values to reset the filters to.
 */
static inline void reset_butterworth_4_complementary_array(uint8_t n,
    Butterworth4Complementary filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    reset_butterworth_4_complementary(&filter_array[i], value_array[i]);
  }
}

/**
 * @brief Get current outputs from an array of 4th order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth4Complementary filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_4_complementary_array(uint8_t n,
    const Butterworth4Complementary filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    output_array[i] = get_butterworth_4_complementary(&filter_array[i]);
  }
}

#endif /* COMPLEMENTARY_FILTER_TYPES_H */
