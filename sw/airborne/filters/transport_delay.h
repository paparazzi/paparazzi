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
/** @file transport_delay.h
 *  @brief Transport delay filter implementation.
 */

#ifndef TRANSPORT_DELAY_H
#define TRANSPORT_DELAY_H

#include "paparazzi.h"

#define TRANSPORT_DELAY_BUFFER_SIZE 20

struct TransportDelay {
  uint8_t delay_samples; // Number of samples to delay
  uint8_t write_index; // Current write index
  float buffer[TRANSPORT_DELAY_BUFFER_SIZE];
};


/**
 * Initialize a transport delay buffer.
 *
 * @param[out] td Pointer to the transport_delay_t structure to initialize.
 * @param[in] delay_samples Number of samples to delay. If this value exceeds TRANSPORT_DELAY_BUFFER_SIZE,
 *        it will be clamped to TRANSPORT_DELAY_BUFFER_SIZE.
 * @param[in] initial_value Initial value to fill the buffer with.
 *
 * @note: If delay_samples > TRANSPORT_DELAY_BUFFER_SIZE, it will be clamped to TRANSPORT_DELAY_BUFFER_SIZE
 *       and the requested delay will not be fully honored.
 */
static inline void init_transport_delay(struct TransportDelay *td, uint8_t delay_samples, const float initial_value)
{
  if (delay_samples > TRANSPORT_DELAY_BUFFER_SIZE) {
    delay_samples = TRANSPORT_DELAY_BUFFER_SIZE;
  }
  td->delay_samples = delay_samples;
  td->write_index = 0;
  for (uint8_t i = 0; i < TRANSPORT_DELAY_BUFFER_SIZE; i++) {
    td->buffer[i] = initial_value;
  }
}

/**
 * Propagate a new input value through the transport delay buffer.
 *
 * @param[in,out] td Pointer to the transport_delay_t structure.
 * @param[in] input New input value to add to the buffer.
 * @return Delayed output value from the buffer.
 */
static inline float update_transport_delay(struct TransportDelay *td, const float input)
{
  td->buffer[td->write_index] = input;
  uint8_t read_index = (td->write_index + TRANSPORT_DELAY_BUFFER_SIZE - td->delay_samples) % TRANSPORT_DELAY_BUFFER_SIZE;
  float output = td->buffer[read_index];
  td->write_index = (td->write_index + 1) % TRANSPORT_DELAY_BUFFER_SIZE;
  return output;
}

/**
 * Reset the transport delay buffer to a specific initial value.
 *
 * @param[in,out] td Pointer to the transport_delay_t structure.
 * @param[in] initial_value Value to reset the buffer elements to.
 */
static inline void reset_transport_delay(struct TransportDelay *td, const float initial_value)
{
  td->write_index = 0;
  for (uint8_t i = 0; i < TRANSPORT_DELAY_BUFFER_SIZE; i++) {
    td->buffer[i] = initial_value;
  }
}

/**
 * Get the current output value from the transport delay buffer without updating it.
 *
 * @param td Pointer to the transport_delay_t structure.
 * @return Current delayed output value from the buffer.
 */
static inline float get_transport_delay(const struct TransportDelay *td)
{
  uint8_t read_index = (td->write_index + TRANSPORT_DELAY_BUFFER_SIZE - td->delay_samples - 1) %
                       TRANSPORT_DELAY_BUFFER_SIZE;
  return td->buffer[read_index];
}

#endif // TRANSPORT_DELAY_H