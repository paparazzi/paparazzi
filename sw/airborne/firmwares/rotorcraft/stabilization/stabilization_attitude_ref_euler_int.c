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

/** @file stabilization_attitude_ref_euler_int.c
 *  Rotorcraft attitude reference generation (euler int version)
 *
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_int.h"
#include "generated/airframe.h"


#define F_UPDATE_RES 9
#define F_UPDATE   (1<<F_UPDATE_RES)

#define REF_ACCEL_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_PDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_QDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_RDOT, REF_ACCEL_FRAC)

#define REF_RATE_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_P, REF_RATE_FRAC)
#define REF_RATE_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_Q, REF_RATE_FRAC)
#define REF_RATE_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_R, REF_RATE_FRAC)

#define OMEGA_P   STABILIZATION_ATTITUDE_REF_OMEGA_P
#define ZETA_P    STABILIZATION_ATTITUDE_REF_ZETA_P
#define ZETA_OMEGA_P_RES 10
#define ZETA_OMEGA_P BFP_OF_REAL((ZETA_P*OMEGA_P), ZETA_OMEGA_P_RES)
#define OMEGA_2_P_RES 7
#define OMEGA_2_P    BFP_OF_REAL((OMEGA_P*OMEGA_P), OMEGA_2_P_RES)

#define OMEGA_Q   STABILIZATION_ATTITUDE_REF_OMEGA_Q
#define ZETA_Q    STABILIZATION_ATTITUDE_REF_ZETA_Q
#define ZETA_OMEGA_Q_RES 10
#define ZETA_OMEGA_Q BFP_OF_REAL((ZETA_Q*OMEGA_Q), ZETA_OMEGA_Q_RES)
#define OMEGA_2_Q_RES 7
#define OMEGA_2_Q    BFP_OF_REAL((OMEGA_Q*OMEGA_Q), OMEGA_2_Q_RES)

#define OMEGA_R   STABILIZATION_ATTITUDE_REF_OMEGA_R
#define ZETA_R    STABILIZATION_ATTITUDE_REF_ZETA_R
#define ZETA_OMEGA_R_RES 10
#define ZETA_OMEGA_R BFP_OF_REAL((ZETA_R*OMEGA_R), ZETA_OMEGA_R_RES)
#define OMEGA_2_R_RES 7
#define OMEGA_2_R    BFP_OF_REAL((OMEGA_R*OMEGA_R), OMEGA_2_R_RES)


#define REF_ANGLE_PI      BFP_OF_REAL(3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define REF_ANGLE_TWO_PI  BFP_OF_REAL(2.*3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define ANGLE_REF_NORMALIZE(_a) {                       \
    while (_a >  REF_ANGLE_PI)  _a -= REF_ANGLE_TWO_PI; \
    while (_a < -REF_ANGLE_PI)  _a += REF_ANGLE_TWO_PI; \
  }



void attitude_ref_euler_int_init(struct AttRefEulerInt *ref)
{
  INT_EULERS_ZERO(ref->euler);
  INT_RATES_ZERO(ref->rate);
  INT_RATES_ZERO(ref->accel);
}

/**
 * Propagate reference model.
 * FIXME: has hardcoded timestep (512Hz via fixed F_UPDATE_RES)
 */
void attitude_ref_euler_int_update(struct AttRefEulerInt *ref, struct Int32Eulers *sp_euler)
{

  /* dumb integrate reference attitude        */
  const struct Int32Eulers d_angle = {
    ref->rate.p >> (F_UPDATE_RES + REF_RATE_FRAC - REF_ANGLE_FRAC),
    ref->rate.q >> (F_UPDATE_RES + REF_RATE_FRAC - REF_ANGLE_FRAC),
    ref->rate.r >> (F_UPDATE_RES + REF_RATE_FRAC - REF_ANGLE_FRAC)
  };
  EULERS_ADD(ref->euler, d_angle);
  ANGLE_REF_NORMALIZE(ref->euler.psi);

  /* integrate reference rotational speeds   */
  const struct Int32Rates d_rate = {
    ref->accel.p >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
    ref->accel.q >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC),
    ref->accel.r >> (F_UPDATE_RES + REF_ACCEL_FRAC - REF_RATE_FRAC)
  };
  RATES_ADD(ref->rate, d_rate);

  /* attitude setpoint with REF_ANGLE_FRAC   */
  struct Int32Eulers sp_ref;
  INT32_EULERS_LSHIFT(sp_ref, *sp_euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));

  /* compute reference attitude error        */
  struct Int32Eulers ref_err;
  EULERS_DIFF(ref_err, ref->euler, sp_ref);
  /* wrap it in the shortest direction       */
  ANGLE_REF_NORMALIZE(ref_err.psi);

  /* compute reference angular accelerations */
  const struct Int32Rates accel_rate = {
    ((int32_t)(-2.*ZETA_OMEGA_P) * (ref->rate.p >> (REF_RATE_FRAC - REF_ACCEL_FRAC)))
        >> (ZETA_OMEGA_P_RES),
    ((int32_t)(-2.*ZETA_OMEGA_Q) * (ref->rate.q >> (REF_RATE_FRAC - REF_ACCEL_FRAC)))
    >> (ZETA_OMEGA_Q_RES),
    ((int32_t)(-2.*ZETA_OMEGA_R) * (ref->rate.r >> (REF_RATE_FRAC - REF_ACCEL_FRAC)))
    >> (ZETA_OMEGA_R_RES)
  };

  const struct Int32Rates accel_angle = {
    ((int32_t)(-OMEGA_2_P) * (ref_err.phi   >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_P_RES),
    ((int32_t)(-OMEGA_2_Q) * (ref_err.theta >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_Q_RES),
    ((int32_t)(-OMEGA_2_R) * (ref_err.psi   >> (REF_ANGLE_FRAC - REF_ACCEL_FRAC))) >> (OMEGA_2_R_RES)
  };

  RATES_SUM(ref->accel, accel_rate, accel_angle);

  /*  saturate acceleration */
  const struct Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
  const struct Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
  RATES_BOUND_BOX(ref->accel, MIN_ACCEL, MAX_ACCEL);

  /* saturate speed and trim accel accordingly */
}
