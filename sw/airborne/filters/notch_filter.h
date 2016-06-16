/*
 * Copyright (C) 2016 Bart Slinger
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

/** @file filters/notch_filter.h
 *  @brief Second order notch filter
 *
 */

#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include "std.h"

struct SecondOrderNotchFilter {
  float Ts;
  float d2;
  float costheta;
  int32_t xn1;
  int32_t xn2;
  int32_t yn1;
  int32_t yn2;
};

/** Set sampling frequency of the notch filter
 *
 * @param frequency frequency at which the filter is updated
 */
static inline void notch_filter_set_sampling_frequency(struct SecondOrderNotchFilter *filter, uint16_t frequency)
{
  filter->Ts = 1.0/frequency;
}

/** Set bandwidth of the notch filter
 *
 * @param bandwidth bandwidth of the filter [Hz]
 */
static inline void notch_filter_set_bandwidth(struct SecondOrderNotchFilter *filter, float bandwidth)
{
  float d = expf(-M_PI*bandwidth*filter->Ts);
  filter->d2 = d*d;
}

/** Set notch filter frequency in Hz
 *
 * @param frequency to attenuate [Hz]
 */
static inline void notch_filter_set_filter_frequency(struct SecondOrderNotchFilter *filter, float frequency)
{
  float theta = 2.0*M_PI*frequency*filter->Ts;
  filter->costheta = cosf(theta);
}

/** Notch filter propagate
 *
 * Discrete implementation:
 * y[n] = b * y[n-1] - d^2 * y[n-2] + a * x[n] - b * x[n-1] + a * x[n-2]
 *
 * @param input_signal input x[n]
 * @param output_signal output y[n]
 */
static inline void notch_filter_update(struct SecondOrderNotchFilter *filter, int32_t *input_signal, int32_t *output_signal)
{
  float a = (1 + filter->d2) * 0.5;
  float b = (1 + filter->d2) * filter->costheta;
  *output_signal = (b * filter->yn1) - (filter->d2 * filter->yn2) + (a * *input_signal) - (b * filter->xn1) + (a * filter->xn2);

  /* Update values for next update */
  filter->xn2 = filter->xn1;
  filter->xn1 = *input_signal;
  filter->yn2 = filter->yn1;
  filter->yn1 = *output_signal;
}

/** Initialize second order notch filter
 *
 * Discrete implementation:
 * y[n] = b * y[n-1] - d^2 * y[n-2] + a * x[n] - b * x[n-1] + a * x[n-2]
 *
 * @param cutoff_frequency frequency to attenuate [Hz]
 * @param bandwidth bandwidth of the filter [Hz]
 * @param sample_frequency frequency at which the filter is updated
 */
static inline void notch_filter_init(struct SecondOrderNotchFilter *filter, float cutoff_frequency, float bandwidth, uint16_t sample_frequency)
{
  notch_filter_set_sampling_frequency(filter, sample_frequency);
  notch_filter_set_filter_frequency(filter, cutoff_frequency);
  notch_filter_set_bandwidth(filter, bandwidth);
  filter->xn1 = 0;
  filter->xn2 = 0;
  filter->yn1 = 0;
  filter->yn2 = 0;
}

#endif
