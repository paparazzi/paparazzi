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
 * @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h
 *
 * Rotorcraft attitude reference generation.
 * (quaternion int version)
 *
 */

#ifndef STABILIZATION_ATTITUDE_REF_QUAT_INT_H
#define STABILIZATION_ATTITUDE_REF_QUAT_INT_H

#include "stabilization_attitude_ref_int.h"
#include "attitude_ref_saturate_naive.h"

/* ref model is in float and then used to precompute ref values in int */
#include "math/pprz_algebra_float.h"

/** Attitude reference model parameters (quat int) */
struct IntRefModel {
  struct FloatRates omega;
  struct FloatRates zeta;
  /* cached intermediate values in int */
  struct Int32Rates two_zeta_omega;
  struct Int32Rates two_omega2;
};

/** Attitude reference models and state/output (quat int) */
struct AttRefQuatInt {
  struct Int32Eulers euler;   ///< with #INT32_ANGLE_FRAC
  struct Int32Quat   quat;
  struct Int32Rates  rate;    ///< with #REF_RATE_FRAC
  struct Int32Rates  accel;   ///< with #REF_ACCEL_FRAC
  struct IntRefModel model;
  struct Int32RefSat saturation;
};

extern void attitude_ref_quat_int_init(struct AttRefQuatInt *ref);
extern void attitude_ref_quat_int_enter(struct AttRefQuatInt *ref, struct Int32Quat *state_quat);
extern void attitude_ref_quat_int_update(struct AttRefQuatInt *ref, struct Int32Quat *sp_quat, float dt);

extern void attitude_ref_quat_int_set_omega(struct AttRefQuatInt *ref, struct FloatRates *omega);
extern void attitude_ref_quat_int_set_omega_p(struct AttRefQuatInt *ref, float omega_p);
extern void attitude_ref_quat_int_set_omega_q(struct AttRefQuatInt *ref, float omega_q);
extern void attitude_ref_quat_int_set_omega_r(struct AttRefQuatInt *ref, float omega_r);

extern void attitude_ref_quat_int_set_zeta(struct AttRefQuatInt *ref, struct FloatRates *zeta);
extern void attitude_ref_quat_int_set_zeta_p(struct AttRefQuatInt *ref, float zeta_p);
extern void attitude_ref_quat_int_set_zeta_q(struct AttRefQuatInt *ref, float zeta_q);
extern void attitude_ref_quat_int_set_zeta_r(struct AttRefQuatInt *ref, float zeta_r);

extern void attitude_ref_quat_int_set_max_p(struct AttRefQuatInt *ref, float max_p);
extern void attitude_ref_quat_int_set_max_q(struct AttRefQuatInt *ref, float max_q);
extern void attitude_ref_quat_int_set_max_r(struct AttRefQuatInt *ref, float max_r);
extern void attitude_ref_quat_int_set_max_pdot(struct AttRefQuatInt *ref, float max_pdot);
extern void attitude_ref_quat_int_set_max_qdot(struct AttRefQuatInt *ref, float max_qdot);
extern void attitude_ref_quat_int_set_max_rdot(struct AttRefQuatInt *ref, float max_rdot);

#endif /* STABILIZATION_ATTITUDE_REF_QUAT_INT_H */
