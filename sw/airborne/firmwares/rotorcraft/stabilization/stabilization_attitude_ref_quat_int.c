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


struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
struct Int32Eulers stab_att_ref_euler;
struct Int32Quat   stab_att_ref_quat;
struct Int32Rates  stab_att_ref_rate;
struct Int32Rates  stab_att_ref_accel;

struct FloatRefModel stab_att_ref_model = {
  {STABILIZATION_ATTITUDE_REF_OMEGA_P, STABILIZATION_ATTITUDE_REF_OMEGA_Q, STABILIZATION_ATTITUDE_REF_OMEGA_R},
  {STABILIZATION_ATTITUDE_REF_ZETA_P, STABILIZATION_ATTITUDE_REF_ZETA_Q, STABILIZATION_ATTITUDE_REF_ZETA_R}
};

#define TWO_ZETA_OMEGA_RES 10
#define TWO_OMEGA_2_RES 7
static struct Int32Rates two_zeta_omega;
static struct Int32Rates two_omega_2;

static void update_ref_model_p(void)
{
  two_zeta_omega.p = BFP_OF_REAL((2 * stab_att_ref_model.zeta.p * stab_att_ref_model.omega.p), TWO_ZETA_OMEGA_RES);
  two_omega_2.p = BFP_OF_REAL((2 * stab_att_ref_model.omega.p * stab_att_ref_model.omega.p), TWO_OMEGA_2_RES);
}

static void update_ref_model_q(void)
{
  two_zeta_omega.q = BFP_OF_REAL((2 * stab_att_ref_model.zeta.q * stab_att_ref_model.omega.q), TWO_ZETA_OMEGA_RES);
  two_omega_2.q = BFP_OF_REAL((2 * stab_att_ref_model.omega.q * stab_att_ref_model.omega.q), TWO_OMEGA_2_RES);
}

static void update_ref_model_r(void)
{
  two_zeta_omega.r = BFP_OF_REAL((2 * stab_att_ref_model.zeta.r * stab_att_ref_model.omega.r), TWO_ZETA_OMEGA_RES);
  two_omega_2.r = BFP_OF_REAL((2 * stab_att_ref_model.omega.r * stab_att_ref_model.omega.r), TWO_OMEGA_2_RES);
}

static void update_ref_model(void)
{
  update_ref_model_p();
  update_ref_model_q();
  update_ref_model_r();
}


void stabilization_attitude_ref_set_omega_p(float omega_p)
{
  stab_att_ref_model.omega.p = omega_p;
  update_ref_model_p();
}

void stabilization_attitude_ref_set_omega_q(float omega_q)
{
  stab_att_ref_model.omega.q = omega_q;
  update_ref_model_q();
}

void stabilization_attitude_ref_set_omega_r(float omega_r)
{
  stab_att_ref_model.omega.r = omega_r;
  update_ref_model_r();
}

void stabilization_attitude_ref_set_omega(struct FloatRates *omega)
{
  stabilization_attitude_ref_set_omega_p(omega->p);
  stabilization_attitude_ref_set_omega_q(omega->q);
  stabilization_attitude_ref_set_omega_r(omega->r);
}

void stabilization_attitude_ref_set_zeta_p(float zeta_p)
{
  stab_att_ref_model.zeta.p = zeta_p;
  update_ref_model_p();
}

void stabilization_attitude_ref_set_zeta_q(float zeta_q)
{
  stab_att_ref_model.zeta.q = zeta_q;
  update_ref_model_q();
}

void stabilization_attitude_ref_set_zeta_r(float zeta_r)
{
  stab_att_ref_model.zeta.r = zeta_r;
  update_ref_model_r();
}

void stabilization_attitude_ref_set_zeta(struct FloatRates *zeta)
{
  stabilization_attitude_ref_set_zeta_p(zeta->p);
  stabilization_attitude_ref_set_zeta_q(zeta->q);
  stabilization_attitude_ref_set_zeta_r(zeta->r);
}


static inline void reset_psi_ref_from_body(void)
{
  //sp has been set from body using stabilization_attitude_get_yaw_i, use that value
  stab_att_ref_euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
  stab_att_ref_rate.r = 0;
  stab_att_ref_accel.r = 0;
}

void stabilization_attitude_ref_init(void)
{

  INT_EULERS_ZERO(stab_att_sp_euler);
  int32_quat_identity(&stab_att_sp_quat);
  INT_EULERS_ZERO(stab_att_ref_euler);
  int32_quat_identity(&stab_att_ref_quat);
  INT_RATES_ZERO(stab_att_ref_rate);
  INT_RATES_ZERO(stab_att_ref_accel);

  update_ref_model();

}

void stabilization_attitude_ref_enter(void)
{
  reset_psi_ref_from_body();

  /* convert reference attitude with REF_ANGLE_FRAC to eulers with normal INT32_ANGLE_FRAC */
  struct Int32Eulers ref_eul;
  INT32_EULERS_RSHIFT(ref_eul, stab_att_ref_euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
  int32_quat_of_eulers(&stab_att_ref_quat, &ref_eul);
  int32_quat_wrap_shortest(&stab_att_ref_quat);

  /* set reference rate and acceleration to zero */
  memset(&stab_att_ref_accel, 0, sizeof(struct Int32Rates));
  memset(&stab_att_ref_rate, 0, sizeof(struct Int32Rates));
}

/*
 * Reference
 */
#define DT_UPDATE (1./PERIODIC_FREQUENCY)
// CAUTION! Periodic frequency is assumed to be 512 Hz
// which is equal to >> 9
#define F_UPDATE_RES 9

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))


void stabilization_attitude_ref_update(void)
{

  /* integrate reference attitude            */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC))
  };
  struct Int32Quat qdot;
  int32_quat_derivative(&qdot, &rate_ref_scaled, &stab_att_ref_quat);
  //QUAT_SMUL(qdot, qdot, DT_UPDATE);
  qdot.qi = qdot.qi >> F_UPDATE_RES;
  qdot.qx = qdot.qx >> F_UPDATE_RES;
  qdot.qy = qdot.qy >> F_UPDATE_RES;
  qdot.qz = qdot.qz >> F_UPDATE_RES;
  QUAT_ADD(stab_att_ref_quat, qdot);
  int32_quat_normalize(&stab_att_ref_quat);

  /* integrate reference rotational speeds
   * delta rate = ref_accel * dt
   * ref_rate = old_ref_rate + delta_rate
   */
  const struct Int32Rates delta_rate = {
    stab_att_ref_accel.p >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
    stab_att_ref_accel.q >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
    stab_att_ref_accel.r >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC)
  };
  RATES_ADD(stab_att_ref_rate, delta_rate);

  /* compute reference angular accelerations */
  struct Int32Quat err;
  /* compute reference attitude error        */
  int32_quat_inv_comp(&err, &stab_att_sp_quat, &stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&err);

  /* propagate the 2nd order linear model : accel = -2*zeta*omega * rate - omega^2 * angle  */

  const struct Int32Rates accel_rate = {
    (-two_zeta_omega.p * (stab_att_ref_rate.p >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
    (-two_zeta_omega.q * (stab_att_ref_rate.q >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES),
    (-two_zeta_omega.r * (stab_att_ref_rate.r >> (REF_RATE_FRAC - REF_ACCEL_FRAC))) >> (TWO_ZETA_OMEGA_RES)
  };

  /* since error quaternion contains the half-angles we get 2*omega^2*err */
  const struct Int32Rates accel_angle = {
    (-two_omega_2.p * (err.qx >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
    (-two_omega_2.q * (err.qy >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES),
    (-two_omega_2.r * (err.qz >> (INT32_QUAT_FRAC - REF_ACCEL_FRAC))) >> (TWO_OMEGA_2_RES)
  };

  RATES_SUM(stab_att_ref_accel, accel_rate, accel_angle);


  /* saturate acceleration */
  const struct Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
  const struct Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
  RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);

  /* saturate angular speed and trim accel accordingly */
  SATURATE_SPEED_TRIM_ACCEL();


  /* compute ref_euler for debugging and telemetry */
  struct Int32Eulers ref_eul;
  int32_eulers_of_quat(&ref_eul, &stab_att_ref_quat);
  INT32_EULERS_LSHIFT(stab_att_ref_euler, ref_eul, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
}
