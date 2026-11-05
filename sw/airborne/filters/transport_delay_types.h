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
/** @file transport_delay_types.h
 *  @brief Transport delay filter type definitions and array operations.
 */

#ifndef TRANSPORT_DELAY_TYPES_H
#define TRANSPORT_DELAY_TYPES_H

#include "std.h"
#include "paparazzi.h"
#include "filters/transport_delay.h"

/**
 * @brief Initialize an array of TransportDelay structures.
 * @param[in] n Number of TransportDelay structures in the array.
 * @param[in,out] td_array Array of TransportDelay structures to initialize.
 * @param[in] delay_samples Array of delay samples for each TransportDelay structure.
 * @param[in] initial_value Array of initial values to fill the buffers.
 */
static inline void init_transport_delay_array(uint8_t n, struct TransportDelay td_array[restrict n],
    const uint8_t delay_samples[restrict n], float initial_value[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    init_transport_delay(&td_array[i], delay_samples[i], initial_value[i]);
  }
}

/**
 * @brief Update an array of TransportDelay structures with input values.
 * @param[in] n Number of TransportDelay structures in the array.
 * @param[in,out] td_array Array of TransportDelay structures to update.
 * @param[in] input_array Array of input values for each TransportDelay structure.
 */
static inline void update_transport_delay_array(uint8_t n, struct TransportDelay td_array[restrict n],
    const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    update_transport_delay(&td_array[i], input_array[i]);
  }
}

/**
 * @brief Reset an array of TransportDelay structures to specific initial values.
 * @param[in] n Number of TransportDelay structures in the array.
 * @param[in,out] td_array Array of TransportDelay structures to reset.
 * @param[in] initial_value Array of initial values to reset the buffers to.
 */
static inline void reset_transport_delay_array(const uint8_t n, struct TransportDelay td_array[restrict n],
    const float initial_value[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    reset_transport_delay(&td_array[i], initial_value[i]);
  }
}

/**
 * @brief Get output values from an array of TransportDelay structures.
 * @param[in] n Number of TransportDelay structures in the array.
 * @param[in] td_array Array of TransportDelay structures to get outputs from.
 * @param[out] output_array Array to store the output values for each TransportDelay structure.
 */
static inline void get_transport_delay_array(const uint8_t n, const struct TransportDelay td_array[restrict n],
    float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++) {
    output_array[i] = get_transport_delay(&td_array[i]);
  }
}

#endif // TRANSPORT_DELAY_TYPES_H