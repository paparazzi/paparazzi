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
 * @file stabilization_attitude_ref_euler_float.c
 *
 * Rotorcraft attitude reference generation in euler float version.
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_defaults.h"

static inline void reset_psi_ref(struct AttRefEulerFloat *ref, float psi);

/*
 *
 * Implementation.
 * Should not rely on any global variables, so these functions can be used like a lib.
 *
 */
void attitude_ref_euler_float_init(struct AttRefEulerFloat *ref)
{
  FLOAT_EULERS_ZERO(ref->euler);
  FLOAT_RATES_ZERO(ref->rate);
  FLOAT_RATES_ZERO(ref->accel);

  ref->model.omega.p = STABILIZATION_ATTITUDE_REF_OMEGA_P;
  ref->model.omega.q = STABILIZATION_ATTITUDE_REF_OMEGA_Q;
  ref->model.omega.r = STABILIZATION_ATTITUDE_REF_OMEGA_R;
  ref->model.zeta.p = STABILIZATION_ATTITUDE_REF_ZETA_P;
  ref->model.zeta.q = STABILIZATION_ATTITUDE_REF_ZETA_Q;
  ref->model.zeta.r = STABILIZATION_ATTITUDE_REF_ZETA_R;

  ref->saturation.max_rate.p = STABILIZATION_ATTITUDE_REF_MAX_P;
  ref->saturation.max_rate.q = STABILIZATION_ATTITUDE_REF_MAX_Q;
  ref->saturation.max_rate.r = STABILIZATION_ATTITUDE_REF_MAX_R;
  ref->saturation.max_accel.p = STABILIZATION_ATTITUDE_REF_MAX_PDOT;
  ref->saturation.max_accel.q = STABILIZATION_ATTITUDE_REF_MAX_QDOT;
  ref->saturation.max_accel.r = STABILIZATION_ATTITUDE_REF_MAX_RDOT;
}

void attitude_ref_euler_float_enter(struct AttRefEulerFloat *ref, float psi)
{
  reset_psi_ref(ref, psi);
}

void attitude_ref_euler_float_update(struct AttRefEulerFloat *ref, struct FloatEulers *sp_eulers, float dt)
{

  /* dumb integrate reference attitude        */
  struct FloatRates delta_rate;
  RATES_SMUL(delta_rate, ref->rate, dt);
  struct FloatEulers delta_angle;
  EULERS_ASSIGN(delta_angle, delta_rate.p, delta_rate.q, delta_rate.r);
  EULERS_ADD(ref->euler, delta_angle);
  FLOAT_ANGLE_NORMALIZE(ref->euler.psi);

  /* integrate reference rotational speeds   */
  struct FloatRates delta_accel;
  RATES_SMUL(delta_accel, ref->accel, dt);
  RATES_ADD(ref->rate, delta_accel);

  /* compute reference attitude error        */
  struct FloatEulers ref_err;
  EULERS_DIFF(ref_err, ref->euler, *sp_eulers);
  /* wrap it in the shortest direction       */
  FLOAT_ANGLE_NORMALIZE(ref_err.psi);

  /* compute reference angular accelerations -2*zeta*omega*rate - omega*omega*ref_err */
  ref->accel.p = -2. * ref->model.zeta.p * ref->model.omega.p * ref->rate.p -
    ref->model.omega.p * ref->model.omega.p * ref_err.phi;
  ref->accel.q = -2. * ref->model.zeta.q * ref->model.omega.p * ref->rate.q -
    ref->model.omega.q * ref->model.omega.q * ref_err.theta;
  ref->accel.r = -2. * ref->model.zeta.r * ref->model.omega.p * ref->rate.r -
    ref->model.omega.r * ref->model.omega.r * ref_err.psi;

  /*  saturate acceleration */
  attitude_ref_float_saturate_naive(&ref->rate, &ref->accel, &ref->saturation);
}

/*
 *
 * Local helper functions.
 *
 */
static inline void reset_psi_ref(struct AttRefEulerFloat *ref, float psi)
{
  ref->euler.psi = psi;
  ref->rate.r = 0;
  ref->accel.r = 0;
}
