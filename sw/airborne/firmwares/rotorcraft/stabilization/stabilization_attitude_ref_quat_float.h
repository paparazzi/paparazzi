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

#include "math/pprz_algebra_float.h"
#include "stabilization_attitude_ref.h"

#ifndef STABILIZATION_ATTITUDE_GAIN_NB
#define STABILIZATION_ATTITUDE_GAIN_NB 1
#endif

#ifndef STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT
#define STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT 0
#endif

extern struct FloatEulers stab_att_sp_euler;
extern struct FloatQuat   stab_att_sp_quat;

/** Attitude reference model parameters (float) */
struct FloatRefModel {
  struct FloatRates omega;
  struct FloatRates zeta;
};

/** Attitude reference models and state/output (float) */
struct AttRefQuatFloat {
  struct FloatEulers euler;
  struct FloatQuat   quat;
  struct FloatRates  rate;
  struct FloatRates  accel;
  struct FloatRefModel  model[];
};

extern struct AttRefQuatFloat att_ref_quat_f;


extern void attitude_ref_quat_float_init(struct AttRefQuatFloat *ref);
extern void attitude_ref_quat_float_update(struct AttRefQuatFloat *ref, struct FloatQuat *sp_quat, float dt);


extern void stabilization_attitude_ref_schedule(uint8_t idx);

extern void stabilization_attitude_ref_idx_set_omega_p(uint8_t idx, float omega);
extern void stabilization_attitude_ref_idx_set_omega_q(uint8_t idx, float omega);
extern void stabilization_attitude_ref_idx_set_omega_r(uint8_t idx, float omega);
extern void stabilization_attitude_ref_set_omega_p(float omega);
extern void stabilization_attitude_ref_set_omega_q(float omega);
extern void stabilization_attitude_ref_set_omega_r(float omega);

#define stabilization_attitude_ref_quat_float_SetOmegaP(_val) { \
    stabilization_attitude_ref_set_omega_p(_val);               \
  }
#define stabilization_attitude_ref_quat_float_SetOmegaQ(_val) { \
    stabilization_attitude_ref_set_omega_q(_val);               \
  }
#define stabilization_attitude_ref_quat_float_SetOmegaR(_val) { \
    stabilization_attitude_ref_set_omega_r(_val);               \
  }

#endif /* STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H */
