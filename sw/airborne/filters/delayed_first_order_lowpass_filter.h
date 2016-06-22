/*
 * Copyright (C) 2015 Bart Slinger
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file filters/delayed_first_order_lowpass_filter.h
 *  @brief First order low-pass filter with delay
 *
 */

#ifndef DELAYED_FIRST_ORDER_LOWPASS_FILTER_H
#define DELAYED_FIRST_ORDER_LOWPASS_FILTER_H

#include "paparazzi.h"

#define DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE 20
#define DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT 14

struct delayed_first_order_lowpass_filter_t {
  uint16_t sample_frequency;
  uint32_t omega;
  uint8_t delay;
  int32_t alpha;
  uint16_t max_inc;
  int32_t buffer[DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE];
  uint8_t idx;
};

/**
 * @brief delayed_first_order_lowpass_propagate
 * @param f Reference to the filter.
 * @param input Value that needs to be filtered.
 * @return alpha*previous + (1 - alpha)*input
 *
 * The actual low-pass filter with delay. Delay is accomplished by internal
 * buffer.
 */
static inline int32_t delayed_first_order_lowpass_propagate(struct delayed_first_order_lowpass_filter_t *f,
    int32_t input)
{
  int32_t prev = f->buffer[f->idx];
  uint8_t next_idx = ++f->idx % DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE;
  f->idx = next_idx;
  f->buffer[next_idx] = (f->alpha * prev + ((1 << DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT) - f->alpha) *
                         input) >> DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT;

  /* Check if new value exceeds maximum increase */
  if ((f->buffer[next_idx] - prev) > f->max_inc) {
    f->buffer[next_idx] = prev + f->max_inc;
  }
  /* Also negative case */
  if ((f->buffer[next_idx] - prev) < -f->max_inc) {
    f->buffer[next_idx] = prev - f->max_inc;
  }

  uint8_t req_idx = (f->idx - f->delay + DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE) %
                    DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE;
  return f->buffer[req_idx];
}

/**
 * @brief delayed_first_order_lowpass_set_omega
 * @param f Reference to the filter.
 * @param omega Filter bandwidth in [rad/s], only positive integer values.
 *
 * Function to change the bandwidth of the filter, can be done in run-time.
 */
static inline void delayed_first_order_lowpass_set_omega(struct delayed_first_order_lowpass_filter_t *f, uint32_t omega)
{
  /* alpha = 1 / ( 1 + omega_c * Ts) */
  f->omega = omega;
  f->alpha = (f->sample_frequency << DELAYED_FIRST_ORDER_LOWPASS_FILTER_FILTER_ALPHA_SHIFT) /
             (f->sample_frequency + f->omega);
}

/**
 * @brief delayed_first_order_lowpass_set_delay
 * @param f Reference to the filter.
 * @param delay Number of timesteps delay in the signal. Maximum defined by buffer size.
 *
 * Function to change the number of timesteps delay. This can be done during
 * run-time. It basically changes the offset to the buffer value which is
 * returned on a propagation.
 */
static inline void delayed_first_order_lowpass_set_delay(struct delayed_first_order_lowpass_filter_t *f, uint8_t delay)
{
  /* Delay cannot be more than buffer size minus one */
  if (delay >= DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE) {
    f->delay = DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE - 1;
  } else {
    f->delay = delay;
  }
}

/**
 * @brief delayed_first_order_lowpass_initialize
 * @param f Reference to the filter.
 * @param omega Filter bandwidth in [rad/s], only positive integer values.
 * @param delay Number of timesteps delay in the signal. Maximum defined by buffer size.
 * @param sample_frequency Frequency at which the filter is going to be updated.
 *
 * Initializes the filter, should be done before using it.
 */
static inline void delayed_first_order_lowpass_initialize(struct delayed_first_order_lowpass_filter_t *f,
    uint32_t omega, uint8_t delay, uint16_t max_inc, uint16_t sample_frequency)
{
  /* Set sample frequency */
  f->sample_frequency = sample_frequency;
  /* Set delay */
  delayed_first_order_lowpass_set_delay(f, delay);

  /* Set omega and calculate alpha */
  delayed_first_order_lowpass_set_omega(f, omega);

  /* Set maximum increase per cycle */
  f->max_inc = max_inc;

  /* Clear the buffer */
  f->idx = 0;
  for (uint8_t i = 0; i < DELAYED_FIRST_ORDER_LOWPASS_FILTER_BUFFER_SIZE; i++) {
    f->buffer[i] = 0;
  }
}
#endif
