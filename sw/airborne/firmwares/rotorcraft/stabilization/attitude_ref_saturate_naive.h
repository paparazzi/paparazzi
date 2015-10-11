/*
 * Copyright (C) 2008-2015 The Paparazzi Team
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

/** @file firmwares/rotorcraft/stabilization/attitude_ref_saturate_naive.h
 *  Naive attitude reference saturation.
 */

#ifndef ATTITUDE_REF_SATURATE_NAIVE_H
#define ATTITUDE_REF_SATURATE_NAIVE_H

#include "math/pprz_algebra_float.h"

struct FloatRefSat {
  struct FloatRates max_rate;
  struct FloatRates max_accel;
};

/** saturate angular speed and trim accel accordingly */
static inline void saturate_speed_trim_accel(float *rate, float *accel, float max_rate)
{
  if (*rate >= max_rate) {
    *rate = max_rate;
    if (*accel > 0.) {
      *accel = 0.;
    }
  }
  else if (*rate <= -max_rate) {
    *rate = -max_rate;
    if (*accel < 0.) {
      *accel = 0.;
    }
  }
}

static inline void attitude_ref_float_saturate_naive(struct FloatRates *rate,
                                                     struct FloatRates *accel,
                                                     struct FloatRefSat *sat)
{
  /* saturate angular acceleration */
  RATES_BOUND_BOX_ABS(*accel, sat->max_accel);

  /* saturate angular speed and trim accel accordingly */
  saturate_speed_trim_accel(&rate->p, &accel->p, sat->max_rate.p);
  saturate_speed_trim_accel(&rate->q, &accel->q, sat->max_rate.q);
  saturate_speed_trim_accel(&rate->r, &accel->r, sat->max_rate.r);
}

#endif /* ATTITUDE_REF_SATURATE_NAIVE_H */
