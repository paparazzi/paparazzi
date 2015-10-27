/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef STABILIZATION_ATTITUDE_REF_EULER_FLOAT_H
#define STABILIZATION_ATTITUDE_REF_EULER_FLOAT_H

#include "math/pprz_algebra_float.h"
#include "attitude_ref_saturate_naive.h"

/** Attitude reference model parameters (float) */
struct FloatRefModel {
  struct FloatRates omega;
  struct FloatRates zeta;
};

/** Attitude reference state/output (euler float) */
struct AttRefEulerFloat {
  struct FloatEulers euler;
  struct FloatRates  rate;
  struct FloatRates  accel;
  struct FloatRefSat saturation;
  struct FloatRefModel model;
};

extern void attitude_ref_euler_float_init(struct AttRefEulerFloat *ref);
extern void attitude_ref_euler_float_enter(struct AttRefEulerFloat *ref, float psi);
extern void attitude_ref_euler_float_update(struct AttRefEulerFloat *ref, struct FloatEulers *sp_eulers, float dt);

#endif /* STABILIZATION_ATTITUDE_REF_EULER_FLOAT_H */
