/*
 * Copyright (C) 2023 Dennis van Wijngaarden
 *                    Ewoud Smeur
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

/** @file filters/adaptive_notch_filter.h
 *  @brief Adaptive notch filter
 *
 */

#ifndef ADAPTIVE_NOTCH_FILTER_H
#define ADAPTIVE_NOTCH_FILTER_H

#include "std.h"
#include "filters/low_pass_filter.h"
#include "filters/notch_filter_float.h"

struct AdaptiveNotchFilter {
  float Ts;                                       // Sample time
  float lp_cutoff_f;                              // Lowpass cutoff frequency
  float hp_cutoff_f;                              // Highpass cutoff frequency
  float r;                                        // Stop-band filter's bandwidth
  float r2;                                       // Stop-band filter's bandwidth squared
  float lambda;                                   // Learning gain

  float a_est;                                    // estimated notch frequency
  float y[2];                                     // Notch filtered measurement vector
  float y_est[2];                                 // Notch filtered estimation vector

  struct SecondOrderNotchFilter output_n_filter;  // Output notch filter
  Butterworth2LowPass lp_filter;           // Lowpass filter for the lower bound of the bandpass filter
  Butterworth2LowPass hp_filter;           // Lowpass filter for the higher bound of the bandpass filter

  float xband[3];                                 // Bandpassed measurement signals signals
};

/** Init a adaptive notch filter.
 *
 * @param filter adaptive notch filter structure
 * @param sample_time sampling period of the signal
 * @param low_freq_bound lower frequency bound of the adaptive notch filter
 * @param high_freq_bound higher frequency bound of the adaptive notch filter
 * @param bandwidth bandwidth of the adaptive notch filter
 * @param lambda learning rate of the adaptive notch filter
 * @param a_ini initial band-stop frequency of the adaptive notch filter
 * @param value initial value of the filter
 */
static inline void adaptive_notch_filter_init(struct AdaptiveNotchFilter *filter, float sample_time,
    float low_freq_bound, float high_freq_bound, float bandwidth, float lambda, float a_ini, float value)
{
  // Init filter parameters
  filter->Ts = sample_time;
  filter->lp_cutoff_f = low_freq_bound;
  filter->hp_cutoff_f = high_freq_bound;
  filter->r = bandwidth;
  filter->lambda = lambda;

  // Initialize initial filter values
  filter->y[0] = filter->y[1] = filter->y_est[0] = filter->y_est[1] = value;
  filter->a_est = a_ini;

  filter->xband[0] = filter->xband[1] = filter->xband[2] = value;

  // Init output notch filter
  float fs = 1. / filter->Ts;
  notch_filter_init(&filter->output_n_filter, filter->a_est, filter->r, fs);

  // Init lowpass filter
  float lp_tau = 1.0 / (2.0 * M_PI * filter->lp_cutoff_f);
  init_butterworth_2_low_pass(&filter->lp_filter, lp_tau, filter->Ts, 0.0);

  // Init highpass filter
  float hp_tau = 1.0 / (2.0 * M_PI * filter->hp_cutoff_f);
  init_butterworth_2_low_pass(&filter->hp_filter, hp_tau, filter->Ts, 0.0);

}

/** Update a second order adaptive notch filter.
 *
 * @param filter adaptive notch filter structure
 * @param value new input value of the filter
 * @param output pointer to float for output
 * @return new filtered value
 */
static inline void adaptive_notch_filter_update(struct AdaptiveNotchFilter *filter, float value, float *output_signal)
{
  // Propagate Butterworth lowpass and hightpass filters for a bandpass signal
  update_butterworth_2_low_pass(&filter->lp_filter, value);
  update_butterworth_2_low_pass(&filter->hp_filter, value);

  filter->xband[2] = filter->xband[1];
  filter->xband[1] = filter->xband[0];
  filter->xband[0] = value - filter->lp_filter.o[0] - (value - filter->hp_filter.o[0]);

  // Run output notch filter
  notch_filter_set_filter_frequency(&filter->output_n_filter, filter->a_est);
  notch_filter_update(&filter->output_n_filter, &value, output_signal);

  // Run estimation notch filter
  float out_est = filter->xband[0] - filter->a_est * filter->xband[1] + filter->xband[2] + filter->r * filter->a_est *
                  filter->y_est[0] - filter->r2 * filter->y_est[1];

  float beta = -filter->xband[1] + filter->r * filter->y_est[1];
  filter->a_est = filter->a_est - filter->lambda * filter->Ts * filter->y_est[0] * beta;
  // Bound estimation frequency to the bandpass cutoff frequencies to be safe
  Bound(filter->a_est, filter->lp_cutoff_f, filter->hp_cutoff_f);

  filter->y[1] = filter->y[0];
  filter->y[0] = *output_signal;
  filter->y_est[1] = filter->y_est[0];
  filter->y_est[0] = out_est;

}

/** Return the latest second order adaptive notch filter output
 *
 * @param filter adaptive notch filter structure
 * @return latest filtered value
 */

static inline float adaptive_notch_filter_get_output(struct AdaptiveNotchFilter *filter)
{
  return filter->y_est[0];
}


#endif // ADAPTIVE_NOTCH_FILTER_H