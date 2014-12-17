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

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_saturate.h"
#include "generated/airframe.h"

struct FloatEulers stab_att_sp_euler;
struct FloatEulers stab_att_ref_euler;
struct FloatRates  stab_att_ref_rate;
struct FloatRates  stab_att_ref_accel;

void stabilization_attitude_ref_init(void)
{

  FLOAT_EULERS_ZERO(stab_att_sp_euler);
  FLOAT_EULERS_ZERO(stab_att_ref_euler);
  FLOAT_RATES_ZERO(stab_att_ref_rate);
  FLOAT_RATES_ZERO(stab_att_ref_accel);

}


/*
 * Reference
 */
#define DT_UPDATE (1./PERIODIC_FREQUENCY)

#define REF_ACCEL_MAX_P STABILIZATION_ATTITUDE_REF_MAX_PDOT
#define REF_ACCEL_MAX_Q STABILIZATION_ATTITUDE_REF_MAX_QDOT
#define REF_ACCEL_MAX_R STABILIZATION_ATTITUDE_REF_MAX_RDOT

#define REF_RATE_MAX_P STABILIZATION_ATTITUDE_REF_MAX_P
#define REF_RATE_MAX_Q STABILIZATION_ATTITUDE_REF_MAX_Q
#define REF_RATE_MAX_R STABILIZATION_ATTITUDE_REF_MAX_R

#define OMEGA_P   STABILIZATION_ATTITUDE_REF_OMEGA_P
#define OMEGA_Q   STABILIZATION_ATTITUDE_REF_OMEGA_Q
#define OMEGA_R   STABILIZATION_ATTITUDE_REF_OMEGA_R

#define ZETA_P    STABILIZATION_ATTITUDE_REF_ZETA_P
#define ZETA_Q    STABILIZATION_ATTITUDE_REF_ZETA_Q
#define ZETA_R    STABILIZATION_ATTITUDE_REF_ZETA_R


#define USE_REF 1

static inline void reset_psi_ref_from_body(void)
{
  //sp has been set from body using stabilization_attitude_get_yaw_f, use that value
  stab_att_ref_euler.psi = stab_att_sp_euler.psi;
  stab_att_ref_rate.r = 0;
  stab_att_ref_accel.r = 0;
}

void stabilization_attitude_ref_enter()
{
  reset_psi_ref_from_body();
}

void stabilization_attitude_ref_update()
{

#if USE_REF

  /* dumb integrate reference attitude        */
  struct FloatRates delta_rate;
  RATES_SMUL(delta_rate, stab_att_ref_rate, DT_UPDATE);
  struct FloatEulers delta_angle;
  EULERS_ASSIGN(delta_angle, delta_rate.p, delta_rate.q, delta_rate.r);
  EULERS_ADD(stab_att_ref_euler, delta_angle);
  FLOAT_ANGLE_NORMALIZE(stab_att_ref_euler.psi);

  /* integrate reference rotational speeds   */
  struct FloatRates delta_accel;
  RATES_SMUL(delta_accel, stab_att_ref_accel, DT_UPDATE);
  RATES_ADD(stab_att_ref_rate, delta_accel);

  /* compute reference attitude error        */
  struct FloatEulers ref_err;
  EULERS_DIFF(ref_err, stab_att_ref_euler, stab_att_sp_euler);
  /* wrap it in the shortest direction       */
  FLOAT_ANGLE_NORMALIZE(ref_err.psi);

  /* compute reference angular accelerations */
  stab_att_ref_accel.p = -2.*ZETA_P * OMEGA_P * stab_att_ref_rate.p - OMEGA_P * OMEGA_P * ref_err.phi;
  stab_att_ref_accel.q = -2.*ZETA_Q * OMEGA_P * stab_att_ref_rate.q - OMEGA_Q * OMEGA_Q * ref_err.theta;
  stab_att_ref_accel.r = -2.*ZETA_R * OMEGA_P * stab_att_ref_rate.r - OMEGA_R * OMEGA_R * ref_err.psi;

  /*  saturate acceleration */
  const struct FloatRates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
  const struct FloatRates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
  RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);

  /* saturate speed and trim accel accordingly */
  SATURATE_SPEED_TRIM_ACCEL();

#else   /* !USE_REF */
  EULERS_COPY(stab_att_ref_euler, stabilization_att_sp);
  FLOAT_RATES_ZERO(stab_att_ref_rate);
  FLOAT_RATES_ZERO(stab_att_ref_accel);
#endif /* USE_REF */

}
