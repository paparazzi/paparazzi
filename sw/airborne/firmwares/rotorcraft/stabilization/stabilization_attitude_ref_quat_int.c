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
 * @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.c
 *
 * Rotorcraft attitude reference generation.
 * (quaternion int version)
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_defaults.h"


#define TWO_ZETA_OMEGA_RES 10
#define TWO_OMEGA_2_RES 7


static inline void reset_psi_ref(struct AttRefQuatInt *ref, int32_t psi);
static void update_ref_model_p(struct AttRefQuatInt *ref);
static void update_ref_model_q(struct AttRefQuatInt *ref);
static void update_ref_model_r(struct AttRefQuatInt *ref);
static void update_ref_model(struct AttRefQuatInt *ref);

/*
 *
 * Implementation.
 * Should not rely on any global variables, so these functions can be used like a lib.
 *
 */
void attitude_ref_quat_int_init(struct AttRefQuatInt *ref)
{
  INT_EULERS_ZERO(ref->euler);
  int32_quat_identity(&ref->quat);
  INT_RATES_ZERO(ref->rate);
  INT_RATES_ZERO(ref->accel);

  attitude_ref_quat_int_set_max_p(ref, STABILIZATION_ATTITUDE_REF_MAX_P);
  attitude_ref_quat_int_set_max_q(ref, STABILIZATION_ATTITUDE_REF_MAX_Q);
  attitude_ref_quat_int_set_max_r(ref, STABILIZATION_ATTITUDE_REF_MAX_R);
  attitude_ref_quat_int_set_max_pdot(ref, STABILIZATION_ATTITUDE_REF_MAX_PDOT);
  attitude_ref_quat_int_set_max_qdot(ref, STABILIZATION_ATTITUDE_REF_MAX_QDOT);
  attitude_ref_quat_int_set_max_rdot(ref, STABILIZATION_ATTITUDE_REF_MAX_RDOT);

  struct FloatRates omega0 = {STABILIZATION_ATTITUDE_REF_OMEGA_P,
                              STABILIZATION_ATTITUDE_REF_OMEGA_Q,
                              STABILIZATION_ATTITUDE_REF_OMEGA_R};
  struct FloatRates zeta0 = {STABILIZATION_ATTITUDE_REF_ZETA_P,
                             STABILIZATION_ATTITUDE_REF_ZETA_Q,
                             STABILIZATION_ATTITUDE_REF_ZETA_R};
  attitude_ref_quat_int_set_omega(ref, &omega0);
  attitude_ref_quat_int_set_zeta(ref, &zeta0);

  /* calc the intermediate cached values */
  update_ref_model(ref);
}

void attitude_ref_quat_int_enter(struct AttRefQuatInt *ref, int32_t psi)
{
  reset_psi_ref(ref, psi);

  int32_quat_of_eulers(&ref->quat, &ref->euler);
  int32_quat_wrap_shortest(&ref->quat);

  /* set reference rate and acceleration to zero */
  memset(&ref->accel, 0, sizeof(struct Int32Rates));
  memset(&ref->rate, 0, sizeof(struct Int32Rates));
}

// CAUTION! Periodic frequency is assumed to be 512 Hz
// which is equal to >> 9
#define F_UPDATE_RES 9

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
/**
 * Propagate reference.
 * CAUTION! Periodic frequency is assumed to be 512 Hz.
 * FIXME: use dt instead of hardcoded 512Hz via F_UPDATE_RES
 */
void attitude_ref_quat_int_update(struct AttRefQuatInt *ref, struct Int32Quat *sp_quat,
                                  float dt __attribute__((unused)))
{
  /* integrate reference attitude            */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(ref->rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(ref->rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(ref->rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC))
  };
  struct Int32Quat qdot;
  int32_quat_derivative(&qdot, &rate_ref_scaled, &ref->quat);
  qdot.qi = qdot.qi >> F_UPDATE_RES;
  qdot.qx = qdot.qx >> F_UPDATE_RES;
  qdot.qy = qdot.qy >> F_UPDATE_RES;
  qdot.qz = qdot.qz >> F_UPDATE_RES;
  QUAT_ADD(ref->quat, qdot);
  int32_quat_normalize(&ref->quat);

  /* integrate reference rotational speeds
   * delta rate = ref_accel * dt
   * ref_rate = old_ref_rate + delta_rate
   */
  const struct Int32Rates delta_rate = {
    ref->accel.p >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
    ref->accel.q >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
    ref->accel.r >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC)
  };
  RATES_ADD(ref->rate, delta_rate);

  /* compute reference angular accelerations */
  struct Int32Quat err;
  /* compute reference attitude error        */
  int32_quat_inv_comp(&err, sp_quat, &ref->quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&err);

  /* propagate the 2nd order linear model : accel = -2*zeta*omega * rate - omega^2 * angle  */

  const struct Int32Rates accel_rate = {
    (-ref->model.two_zeta_omega.p * (ref->rate.p >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
    (-ref->model.two_zeta_omega.q * (ref->rate.q >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
    (-ref->model.two_zeta_omega.r * (ref->rate.r >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES)
  };

  /* since error quaternion contains the half-angles we get 2*omega^2*err */
  const struct Int32Rates accel_angle = {
    (-ref->model.two_omega2.p * (err.qx >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
    (-ref->model.two_omega2.q * (err.qy >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
    (-ref->model.two_omega2.r * (err.qz >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES)
  };

  RATES_SUM(ref->accel, accel_rate, accel_angle);


  /* saturate */
  attitude_ref_int_saturate_naive(&ref->rate, &ref->accel, &ref->saturation);

  /* compute euler representation for debugging and telemetry */
  int32_eulers_of_quat(&ref->euler, &ref->quat);
}


static inline void reset_psi_ref(struct AttRefQuatInt *ref, int32_t psi)
{
  ref->euler.psi = psi;
  ref->rate.r = 0;
  ref->accel.r = 0;
}


/*
 * Recomputation of cached values.
 *
 */
static void update_ref_model_p(struct AttRefQuatInt *ref)
{
  ref->model.two_zeta_omega.p = BFP_OF_REAL((2 * ref->model.zeta.p * ref->model.omega.p), TWO_ZETA_OMEGA_RES);
  ref->model.two_omega2.p = BFP_OF_REAL((2 * ref->model.omega.p * ref->model.omega.p), TWO_OMEGA_2_RES);
}

static void update_ref_model_q(struct AttRefQuatInt *ref)
{
  ref->model.two_zeta_omega.q = BFP_OF_REAL((2 * ref->model.zeta.q * ref->model.omega.q), TWO_ZETA_OMEGA_RES);
  ref->model.two_omega2.q = BFP_OF_REAL((2 * ref->model.omega.q * ref->model.omega.q), TWO_OMEGA_2_RES);
}

static void update_ref_model_r(struct AttRefQuatInt *ref)
{
  ref->model.two_zeta_omega.r = BFP_OF_REAL((2 * ref->model.zeta.r * ref->model.omega.r), TWO_ZETA_OMEGA_RES);
  ref->model.two_omega2.r = BFP_OF_REAL((2 * ref->model.omega.r * ref->model.omega.r), TWO_OMEGA_2_RES);
}

static void update_ref_model(struct AttRefQuatInt *ref)
{
  update_ref_model_p(ref);
  update_ref_model_q(ref);
  update_ref_model_r(ref);
}


/*
 * Setting handlers for changing the ref model parameters.
 *
 */
void attitude_ref_quat_int_set_omega_p(struct AttRefQuatInt *ref, float omega_p)
{
  ref->model.omega.p = omega_p;
  update_ref_model_p(ref);
}

void attitude_ref_quat_int_set_omega_q(struct AttRefQuatInt *ref, float omega_q)
{
  ref->model.omega.q = omega_q;
  update_ref_model_q(ref);
}

void attitude_ref_quat_int_set_omega_r(struct AttRefQuatInt *ref, float omega_r)
{
  ref->model.omega.r = omega_r;
  update_ref_model_r(ref);
}

void attitude_ref_quat_int_set_omega(struct AttRefQuatInt *ref, struct FloatRates *omega)
{
  attitude_ref_quat_int_set_omega_p(ref, omega->p);
  attitude_ref_quat_int_set_omega_q(ref, omega->q);
  attitude_ref_quat_int_set_omega_r(ref, omega->r);
}

void attitude_ref_quat_int_set_zeta_p(struct AttRefQuatInt *ref, float zeta_p)
{
  ref->model.zeta.p = zeta_p;
  update_ref_model_p(ref);
}

void attitude_ref_quat_int_set_zeta_q(struct AttRefQuatInt *ref, float zeta_q)
{
  ref->model.zeta.q = zeta_q;
  update_ref_model_q(ref);
}

void attitude_ref_quat_int_set_zeta_r(struct AttRefQuatInt *ref, float zeta_r)
{
  ref->model.zeta.r = zeta_r;
  update_ref_model_r(ref);
}

void attitude_ref_quat_int_set_zeta(struct AttRefQuatInt *ref, struct FloatRates *zeta)
{
  attitude_ref_quat_int_set_zeta_p(ref, zeta->p);
  attitude_ref_quat_int_set_zeta_q(ref, zeta->q);
  attitude_ref_quat_int_set_zeta_r(ref, zeta->r);
}

void attitude_ref_quat_int_set_max_p(struct AttRefQuatInt *ref, float max_p)
{
  ref->saturation.max_rate.p = BFP_OF_REAL(max_p, REF_RATE_FRAC);
}

void attitude_ref_quat_int_set_max_q(struct AttRefQuatInt *ref, float max_q)
{
  ref->saturation.max_rate.q = BFP_OF_REAL(max_q, REF_RATE_FRAC);
}

void attitude_ref_quat_int_set_max_r(struct AttRefQuatInt *ref, float max_r)
{
  ref->saturation.max_rate.r = BFP_OF_REAL(max_r, REF_RATE_FRAC);
}

void attitude_ref_quat_int_set_max_pdot(struct AttRefQuatInt *ref, float max_pdot)
{
  ref->saturation.max_accel.p = BFP_OF_REAL(max_pdot, REF_ACCEL_FRAC);
}

void attitude_ref_quat_int_set_max_qdot(struct AttRefQuatInt *ref, float max_qdot)
{
  ref->saturation.max_accel.q = BFP_OF_REAL(max_qdot, REF_ACCEL_FRAC);
}

void attitude_ref_quat_int_set_max_rdot(struct AttRefQuatInt *ref, float max_rdot)
{
  ref->saturation.max_accel.r = BFP_OF_REAL(max_rdot, REF_ACCEL_FRAC);
}

