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
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_saturate.h"

#define REF_ACCEL_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_PDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_QDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_RDOT, REF_ACCEL_FRAC)

#define REF_RATE_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_P, REF_RATE_FRAC)
#define REF_RATE_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_Q, REF_RATE_FRAC)
#define REF_RATE_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_R, REF_RATE_FRAC)



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

  /* convert reference attitude with REF_ANGLE_FRAC to eulers with normal INT32_ANGLE_FRAC */
  struct Int32Eulers ref_eul;
  INT32_EULERS_RSHIFT(ref_eul, ref->euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
  int32_quat_of_eulers(&ref->quat, &ref_eul);
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


  /* saturate acceleration */
  const struct Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
  const struct Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
  RATES_BOUND_BOX(ref->accel, MIN_ACCEL, MAX_ACCEL);

  /* saturate angular speed and trim accel accordingly */
  SATURATE_SPEED_TRIM_ACCEL(*ref);


  /* compute ref_euler for debugging and telemetry */
  struct Int32Eulers ref_eul;
  int32_eulers_of_quat(&ref_eul, &ref->quat);
  INT32_EULERS_LSHIFT(ref->euler, ref_eul, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
}




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


static inline void reset_psi_ref(struct AttRefQuatInt *ref, int32_t psi)
{
  ref->euler.psi = psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
  ref->rate.r = 0;
  ref->accel.r = 0;
}
