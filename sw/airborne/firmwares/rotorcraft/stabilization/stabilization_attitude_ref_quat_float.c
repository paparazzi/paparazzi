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
 * @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.c
 *
 * Rotorcraft attitude reference generation.
 * (quaternion float version)
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

#include "stabilization_attitude_ref_float.h"

#define REF_ACCEL_MAX_P STABILIZATION_ATTITUDE_REF_MAX_PDOT
#define REF_ACCEL_MAX_Q STABILIZATION_ATTITUDE_REF_MAX_QDOT
#define REF_ACCEL_MAX_R STABILIZATION_ATTITUDE_REF_MAX_RDOT

struct FloatEulers stab_att_sp_euler;
struct FloatQuat   stab_att_sp_quat;
struct FloatEulers stab_att_ref_euler;
struct FloatQuat   stab_att_ref_quat;
struct FloatRates  stab_att_ref_rate;
struct FloatRates  stab_att_ref_accel;

struct FloatRefModel stab_att_ref_model[STABILIZATION_ATTITUDE_GAIN_NB];

static int ref_idx = STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT;

static const float omega_p[] = STABILIZATION_ATTITUDE_REF_OMEGA_P;
static const float zeta_p[] = STABILIZATION_ATTITUDE_REF_ZETA_P;
static const float omega_q[] = STABILIZATION_ATTITUDE_REF_OMEGA_Q;
static const float zeta_q[] = STABILIZATION_ATTITUDE_REF_ZETA_Q;
static const float omega_r[] = STABILIZATION_ATTITUDE_REF_OMEGA_R;
static const float zeta_r[] = STABILIZATION_ATTITUDE_REF_ZETA_R;

static inline void reset_psi_ref_from_body(void) {
  //sp has been set from body using stabilization_attitude_get_yaw_f, use that value
  stab_att_ref_euler.psi = stab_att_sp_euler.psi;
  stab_att_ref_rate.r = 0;
  stab_att_ref_accel.r = 0;
}

static inline void update_ref_quat_from_eulers(void) {
  struct FloatRMat ref_rmat;
  FLOAT_RMAT_OF_EULERS(ref_rmat, stab_att_ref_euler);
  FLOAT_QUAT_OF_RMAT(stab_att_ref_quat, ref_rmat);
  FLOAT_QUAT_WRAP_SHORTEST(stab_att_ref_quat);
}

void stabilization_attitude_ref_init(void) {

  FLOAT_EULERS_ZERO(stab_att_sp_euler);
  FLOAT_QUAT_ZERO(  stab_att_sp_quat);
  FLOAT_EULERS_ZERO(stab_att_ref_euler);
  FLOAT_QUAT_ZERO(  stab_att_ref_quat);
  FLOAT_RATES_ZERO( stab_att_ref_rate);
  FLOAT_RATES_ZERO( stab_att_ref_accel);

  for (int i = 0; i < STABILIZATION_ATTITUDE_GAIN_NB; i++) {
    RATES_ASSIGN(stab_att_ref_model[i].omega, omega_p[i], omega_q[i], omega_r[i]);
    RATES_ASSIGN(stab_att_ref_model[i].zeta, zeta_p[i], zeta_q[i], zeta_r[i]);
  }

}

void stabilization_attitude_ref_schedule(uint8_t idx) {
  ref_idx = idx;
}

void stabilization_attitude_ref_enter(void) {
  reset_psi_ref_from_body();
  update_ref_quat_from_eulers();
}

/*
 * Reference
 */
#define DT_UPDATE (1./PERIODIC_FREQUENCY)

// default to fast but less precise quaternion integration
#ifndef STABILIZATION_ATTITUDE_REF_QUAT_INFINITESIMAL_STEP
#define STABILIZATION_ATTITUDE_REF_QUAT_INFINITESIMAL_STEP TRUE
#endif

void stabilization_attitude_ref_update(void) {

  /* integrate reference attitude            */
#if STABILIZATION_ATTITUDE_REF_QUAT_INFINITESIMAL_STEP
  struct FloatQuat qdot;
  FLOAT_QUAT_DERIVATIVE(qdot, stab_att_ref_rate, stab_att_ref_quat);
  QUAT_SMUL(qdot, qdot, DT_UPDATE);
  QUAT_ADD(stab_att_ref_quat, qdot);
#else // use finite step (involves trig)
  struct FloatQuat delta_q;
  FLOAT_QUAT_DIFFERENTIAL(delta_q, stab_att_ref_rate, DT_UPDATE);
  /* compose new ref_quat by quaternion multiplication of delta rotation and current ref_quat */
  struct FloatQuat new_ref_quat;
  FLOAT_QUAT_COMP(new_ref_quat, delta_q, stab_att_ref_quat);
  QUAT_COPY(stab_att_ref_quat, new_ref_quat);
#endif
  FLOAT_QUAT_NORMALIZE(stab_att_ref_quat);

  /* integrate reference rotational speeds   */
  struct FloatRates delta_rate;
  RATES_SMUL(delta_rate, stab_att_ref_accel, DT_UPDATE);
  RATES_ADD(stab_att_ref_rate, delta_rate);

  /* compute reference angular accelerations */
  struct FloatQuat err;
  /* compute reference attitude error        */
  FLOAT_QUAT_INV_COMP(err, stab_att_sp_quat, stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  FLOAT_QUAT_WRAP_SHORTEST(err);
  /* propagate the 2nd order linear model    */
  stab_att_ref_accel.p = -2.*stab_att_ref_model[ref_idx].zeta.p*stab_att_ref_model[ref_idx].omega.p*stab_att_ref_rate.p
    - stab_att_ref_model[ref_idx].omega.p*stab_att_ref_model[ref_idx].omega.p*err.qx;
  stab_att_ref_accel.q = -2.*stab_att_ref_model[ref_idx].zeta.q*stab_att_ref_model[ref_idx].omega.q*stab_att_ref_rate.q
    - stab_att_ref_model[ref_idx].omega.q*stab_att_ref_model[ref_idx].omega.q*err.qy;
  stab_att_ref_accel.r = -2.*stab_att_ref_model[ref_idx].zeta.r*stab_att_ref_model[ref_idx].omega.r*stab_att_ref_rate.r
    - stab_att_ref_model[ref_idx].omega.r*stab_att_ref_model[ref_idx].omega.r*err.qz;

  /*	saturate acceleration */
  const struct FloatRates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
  const struct FloatRates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
  RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);

  /* compute ref_euler */
  FLOAT_EULERS_OF_QUAT(stab_att_ref_euler, stab_att_ref_quat);
}
