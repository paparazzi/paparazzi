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

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Quat   stab_att_ref_quat;  ///< with #INT32_QUAT_FRAC

void stabilization_attitude_ref_enter(void);

/* ref model is in float and then used to precompute ref values in int */
#include "math/pprz_algebra_float.h"

struct FloatRefModel {
  struct FloatRates omega;
  struct FloatRates zeta;
};

extern struct FloatRefModel stab_att_ref_model;

extern void stabilization_attitude_ref_set_omega(struct FloatRates *omega);
extern void stabilization_attitude_ref_set_omega_p(float omega_p);
extern void stabilization_attitude_ref_set_omega_q(float omega_q);
extern void stabilization_attitude_ref_set_omega_r(float omega_r);

extern void stabilization_attitude_ref_set_zeta(struct FloatRates *zeta);
extern void stabilization_attitude_ref_set_zeta_p(float zeta_p);
extern void stabilization_attitude_ref_set_zeta_q(float zeta_q);
extern void stabilization_attitude_ref_set_zeta_r(float zeta_r);

#define stabilization_attitude_ref_quat_int_SetOmegaP(_val) { \
    stabilization_attitude_ref_set_omega_p(_val);             \
  }
#define stabilization_attitude_ref_quat_int_SetOmegaQ(_val) {   \
    stabilization_attitude_ref_set_omega_q(_val);               \
  }
#define stabilization_attitude_ref_quat_int_SetOmegaR(_val) {   \
    stabilization_attitude_ref_set_omega_r(_val);               \
  }

#define stabilization_attitude_ref_quat_int_SetZetaP(_val) {   \
    stabilization_attitude_ref_set_zeta_p(_val);               \
  }
#define stabilization_attitude_ref_quat_int_SetZetaQ(_val) {    \
    stabilization_attitude_ref_set_zeta_q(_val);                \
  }
#define stabilization_attitude_ref_quat_int_SetZetaR(_val) {    \
    stabilization_attitude_ref_set_zeta_r(_val);                \
  }

#endif /* STABILIZATION_ATTITUDE_REF_QUAT_INT_H */
