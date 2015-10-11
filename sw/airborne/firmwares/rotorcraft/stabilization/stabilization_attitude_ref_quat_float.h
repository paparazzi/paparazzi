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

/**
 * @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.h
 *
 * Rotorcraft attitude reference generation.
 * (quaternion float version)
 *
 */

#ifndef STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H
#define STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H

#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"
#include "attitude_ref_saturate_naive.h"

#ifndef STABILIZATION_ATTITUDE_GAIN_NB
#define STABILIZATION_ATTITUDE_GAIN_NB 1
#endif

/** Attitude reference model parameters (float) */
struct FloatRefModel {
  struct FloatRates omega;
  struct FloatRates zeta;
  /// cached value of 2*omega*omega
  struct FloatRates two_omega2;
};

/** Attitude reference models and state/output (float) */
struct AttRefQuatFloat {
  struct FloatEulers euler;
  struct FloatQuat   quat;
  struct FloatRates  rate;
  struct FloatRates  accel;
  uint8_t cur_idx;
  struct FloatRefModel  model[STABILIZATION_ATTITUDE_GAIN_NB];
  struct FloatRefSat saturation;
};


extern void attitude_ref_quat_float_init(struct AttRefQuatFloat *ref);
extern void attitude_ref_quat_float_enter(struct AttRefQuatFloat *ref, float psi);
extern void attitude_ref_quat_float_update(struct AttRefQuatFloat *ref, struct FloatQuat *sp_quat, float dt);


extern void attitude_ref_quat_float_schedule(struct AttRefQuatFloat *ref, uint8_t idx);

extern void attitude_ref_quat_float_idx_set_omega_p(struct AttRefQuatFloat *ref, uint8_t idx, float omega);
extern void attitude_ref_quat_float_idx_set_omega_q(struct AttRefQuatFloat *ref, uint8_t idx, float omega);
extern void attitude_ref_quat_float_idx_set_omega_r(struct AttRefQuatFloat *ref, uint8_t idx, float omega);
extern void attitude_ref_quat_float_set_omega_p(struct AttRefQuatFloat *ref, float omega);
extern void attitude_ref_quat_float_set_omega_q(struct AttRefQuatFloat *ref, float omega);
extern void attitude_ref_quat_float_set_omega_r(struct AttRefQuatFloat *ref, float omega);


#endif /* STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H */
