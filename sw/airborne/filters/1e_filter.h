/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file filters/1e_filter.h
 *  @brief Implementation of the 1 euro filter
 *
 * See:
 *  http://cristal.univ-lille.fr/~casiez/1euro/
 *
 * Ref:
 *  Casiez, G., Roussel, N. and Vogel, D. (2012). 1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems. Proceedings of the ACM Conference on Human Factors in Computing Systems (CHI '12). Austin, Texas (May 5-12, 2012). New York: ACM Press, pp. 2527-2530.
 *
 */

#ifndef ONE_EURO_FILTER_H
#define ONE_EURO_FILTER_H

#include "std.h"
#include <math.h>

/**
 *  simple low pass filter.
 *  TODO use existing implementation ?
 */
struct OneEuroLPFilter {
  float hatx;
  float hatxprev;
  bool first_time;
};

/**
 *  configuration parameters.
 */
struct OneEuroFilter {
  bool first_time;                ///< first time flag
  float rate;                     ///< data update rate (in Hz)
  float mincutoff;                ///< min cutoff freq (in Hz)
  float beta;                     ///< cutoff slope
  float dcutoff;                  ///< derivative cutoff freq (in Hz)
  struct OneEuroLPFilter xfilt;   ///< low pass filter
  struct OneEuroLPFilter dxfilt;  ///< low pass filter for derivative
  uint32_t last_time;             ///< last update time in ms
};

/**
 *  Initialize a 1 Euro Filter instance.
 */
static inline void init_1e_filter(struct OneEuroFilter *filter, float rate, float mincutoff, float beta, float dcutoff)
{
  filter->first_time = true;
  filter->rate = rate;
  filter->mincutoff = mincutoff;
  filter->beta = beta;
  filter->dcutoff = dcutoff;
  filter->xfilt.hatx = 0.f;
  filter->xfilt.hatxprev = 0.f;
  filter->xfilt.first_time = true;
  filter->dxfilt.hatx = 0.f;
  filter->dxfilt.hatxprev = 0.f;
  filter->dxfilt.first_time = true;
  filter->last_time = 0;
}

/**
 * Reset filter (gains and parameters unchanged)
 */
static inline void reset_1e_filter(struct OneEuroFilter *filter)
{
  filter->first_time = true;
  filter->xfilt.hatx = 0.f;
  filter->xfilt.hatxprev = 0.f;
  filter->xfilt.first_time = true;
  filter->dxfilt.hatx = 0.f;
  filter->dxfilt.hatxprev = 0.f;
  filter->dxfilt.first_time = true;
  filter->last_time = 0;
}

/**
 *  Filter a float using the given low-pass filter and the given alpha value.
 */
static inline float compute_1e_filter_lp(struct OneEuroLPFilter *filter, float x, float alpha)
{
  if (filter->first_time) {
    filter->first_time = false;
    filter->hatxprev = x;
  }
  filter->hatx = alpha * x + (1.f - alpha) * filter->hatxprev;
  filter->hatxprev = filter->hatx;
  return filter->hatx;
}

/**
 *  Compute Alpha for a given One Euro Filter and a given cutoff frequency.
 */
static inline float compute_1e_filter_alpha(struct OneEuroFilter *filter, float cutoff)
{
  float tau = 1.0f / (2.f * M_PI * cutoff);
  float te = 1.0f / filter->rate;
  return 1.0f / (1.0f + tau / te);

}

/**
 *  Filter a float using the given One Euro Filter.
 */
static inline float update_1e_filter(struct OneEuroFilter *filter, float x)
{
  float dx;
  if (filter->first_time) {
    filter->first_time = false;
    dx = 0.f;
  } else {
    dx = (x - filter->xfilt.hatxprev) * filter->rate;
  }
  float edx = compute_1e_filter_lp(&(filter->dxfilt), dx, compute_1e_filter_alpha(filter, filter->dcutoff));
  float cutoff = filter->mincutoff + filter->beta * fabsf(edx);
  return compute_1e_filter_lp(&(filter->xfilt), x, compute_1e_filter_alpha(filter, cutoff));
}

/**
 *  Filter a float using the given One Euro Filter and the given timestamp.
 *  Frequency will be automatically recomputed.
 */
static inline float update_1e_filter_at_time(struct OneEuroFilter *filter, float x, uint32_t timestamp)
{
  if (filter->last_time != 0 && timestamp != filter->last_time) {
    filter->rate = 1e6f / (float)(timestamp - filter->last_time); // timestamp in us
  }
  filter->last_time = timestamp;
  return update_1e_filter(filter, x);
}

#endif

