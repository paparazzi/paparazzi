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
#include "math/pprz_algebra_int.h"

struct FloatRefSat {
  struct FloatRates max_rate;
  struct FloatRates max_accel;
};

struct Int32RefSat {
  struct Int32Rates max_rate;
  struct Int32Rates max_accel;
};

/** saturate angular speed and trim accel accordingly */
#define SATURATE_SPEED_TRIM_ACCEL(_rate, _accel, _max_rate) {   \
    if ((_rate) >= (_max_rate)) {                               \
      (_rate) = (_max_rate);                                    \
      if ((_accel) > 0) {                                       \
        (_accel) = 0;                                           \
      }                                                         \
    }                                                           \
    else if ((_rate) <= -(_max_rate)) {                         \
      (_rate) = -(_max_rate);                                   \
      if ((_accel) < 0) {                                       \
        (_accel) = 0;                                           \
      }                                                         \
    }                                                           \
  }

static inline void attitude_ref_float_saturate_naive(struct FloatRates *rate,
                                                     struct FloatRates *accel,
                                                     struct FloatRefSat *sat)
{
  /* saturate angular acceleration */
  RATES_BOUND_BOX_ABS(*accel, sat->max_accel);

  /* saturate angular speed and trim accel accordingly */
  SATURATE_SPEED_TRIM_ACCEL(rate->p, accel->p, sat->max_rate.p);
  SATURATE_SPEED_TRIM_ACCEL(rate->q, accel->q, sat->max_rate.q);
  SATURATE_SPEED_TRIM_ACCEL(rate->r, accel->r, sat->max_rate.r);
}

static inline void attitude_ref_int_saturate_naive(struct Int32Rates *rate,
                                                   struct Int32Rates *accel,
                                                   struct Int32RefSat *sat)
{
  /* saturate angular acceleration */
  RATES_BOUND_BOX_ABS(*accel, sat->max_accel);

  /* saturate angular speed and trim accel accordingly */
  SATURATE_SPEED_TRIM_ACCEL(rate->p, accel->p, sat->max_rate.p);
  SATURATE_SPEED_TRIM_ACCEL(rate->q, accel->q, sat->max_rate.q);
  SATURATE_SPEED_TRIM_ACCEL(rate->r, accel->r, sat->max_rate.r);
}

#endif /* ATTITUDE_REF_SATURATE_NAIVE_H */
