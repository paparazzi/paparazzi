/*
 * $Id: booz_stabilization_attitude_ref_traj_euler.h 3796 2009-07-25 00:01:02Z poine $
 *  
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

#include "booz_stabilization.h"


struct FloatEulers booz_stab_att_sp_euler;
struct FloatQuat   booz_stab_att_sp_quat;
struct FloatEulers booz_stab_att_ref_euler;
struct FloatQuat   booz_stab_att_ref_quat;
struct FloatRates  booz_stab_att_ref_rate;
struct FloatRates  booz_stab_att_ref_accel;


void booz_stabilization_attitude_ref_init(void) {

  FLOAT_EULERS_ZERO(booz_stab_att_sp_euler);
  FLOAT_QUAT_ZERO(  booz_stab_att_sp_quat);
  FLOAT_EULERS_ZERO(booz_stab_att_ref_euler);
  FLOAT_QUAT_ZERO(  booz_stab_att_ref_quat);
  FLOAT_RATES_ZERO( booz_stab_att_ref_rate);
  FLOAT_RATES_ZERO( booz_stab_att_ref_accel);

}

/*
 * Reference
 */
#ifdef BOOZ_AP_PERIODIC_PRESCALE
#define DT_UPDATE ((float) BOOZ_AP_PERIODIC_PRESCALE / (float) PERIODIC_FREQ)
#else
#define DT_UPDATE (1./512.)
#endif

#define OMEGA_P   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_P
#define ZETA_P    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_P
#define OMEGA_Q   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_Q
#define ZETA_Q    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_Q
#define OMEGA_R   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_R
#define ZETA_R    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_R

void booz_stabilization_attitude_ref_update(bool_t in_flight) {

  /* integrate reference attitude            */
  struct FloatQuat qdot;
  FLOAT_QUAT_DERIVATIVE(qdot, booz_stab_att_ref_rate, booz_stab_att_ref_quat);
  QUAT_SMUL(qdot, qdot, DT_UPDATE);
  QUAT_ADD(booz_stab_att_ref_quat, qdot);
  FLOAT_QUAT_NORMALISE(booz_stab_att_ref_quat);

  /* integrate reference rotational speeds   */
  struct FloatRates delta_rate;
  RATES_SMUL(delta_rate, booz_stab_att_ref_accel, DT_UPDATE);
  RATES_ADD(booz_stab_att_ref_rate, delta_rate);
  
  /* compute reference angular accelerations */
  struct FloatQuat err; 
  /* compute reference attitude error        */
  FLOAT_QUAT_INV_COMP(err, booz_stab_att_sp_quat, booz_stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  FLOAT_QUAT_WRAP_SHORTEST(err);
  /* propagate the 2nd order linear model    */
  booz_stab_att_ref_accel.p = -2.*ZETA_P*OMEGA_P*booz_stab_att_ref_rate.p - OMEGA_P*OMEGA_P*err.qx;
  booz_stab_att_ref_accel.q = -2.*ZETA_Q*OMEGA_Q*booz_stab_att_ref_rate.q - OMEGA_Q*OMEGA_Q*err.qy;

  if (in_flight) {
    booz_stab_att_ref_accel.r = -2.*ZETA_R*OMEGA_R*booz_stab_att_ref_rate.r - OMEGA_R*OMEGA_R*err.qz;
  }

  /* compute ref_euler */
  FLOAT_EULERS_OF_QUAT(booz_stab_att_ref_euler, booz_stab_att_ref_quat);

}


