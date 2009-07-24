/*
 * $Id$
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

#ifndef BOOZ_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_H
#define BOOZ_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_H

#include "booz_radio_control.h"

extern struct Int32Eulers booz_stabilization_att_sp;
extern struct Int32Eulers booz_stabilization_att_ref;
extern struct Int32Vect3  booz_stabilization_rate_ref;
extern struct Int32Vect3  booz_stabilization_accel_ref;

#define F_UPDATE_RES 9
#define F_UPDATE   (1<<F_UPDATE_RES)

#define ACCEL_REF_RES 12
#define ACCEL_REF_MAX_P BOOZ_STABILIZATION_ATTITUDE_REF_MAX_PDOT
#define ACCEL_REF_MAX_Q BOOZ_STABILIZATION_ATTITUDE_REF_MAX_QDOT
#define ACCEL_REF_MAX_R BOOZ_STABILIZATION_ATTITUDE_REF_MAX_RDOT

#define RATE_REF_RES  16
#define RATE_REF_MAX_P BOOZ_STABILIZATION_ATTITUDE_REF_MAX_P
#define RATE_REF_MAX_Q BOOZ_STABILIZATION_ATTITUDE_REF_MAX_Q
#define RATE_REF_MAX_R BOOZ_STABILIZATION_ATTITUDE_REF_MAX_R

#define ANGLE_REF_RES 20
#define PI_ANGLE_REF      BFP_OF_REAL(3.1415926535897932384626433832795029, ANGLE_REF_RES)
#define TWO_PI_ANGLE_REF  BFP_OF_REAL(2.*3.1415926535897932384626433832795029, ANGLE_REF_RES)
#define ANGLE_REF_NORMALIZE(_a) {				\
    while (_a >  PI_ANGLE_REF)  _a -= TWO_PI_ANGLE_REF;		\
    while (_a < -PI_ANGLE_REF)  _a += TWO_PI_ANGLE_REF;		\
  }


#define OMEGA_P   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_P
#define ZETA_P    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_P
#define ZETA_OMEGA_P_RES 10
#define ZETA_OMEGA_P BFP_OF_REAL((ZETA_P*OMEGA_P), ZETA_OMEGA_P_RES)
#define OMEGA_2_P_RES 7
#define OMEGA_2_P    BFP_OF_REAL((OMEGA_P*OMEGA_P), OMEGA_2_P_RES)

#define OMEGA_Q   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_Q
#define ZETA_Q    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_Q
#define ZETA_OMEGA_Q_RES 10
#define ZETA_OMEGA_Q BFP_OF_REAL((ZETA_Q*OMEGA_Q), ZETA_OMEGA_Q_RES)
#define OMEGA_2_Q_RES 7
#define OMEGA_2_Q    BFP_OF_REAL((OMEGA_Q*OMEGA_Q), OMEGA_2_Q_RES)


#define OMEGA_R   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_R
#define ZETA_R    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_R
#define ZETA_OMEGA_R_RES 10
#define ZETA_OMEGA_R BFP_OF_REAL((ZETA_R*OMEGA_R), ZETA_OMEGA_R_RES)
#define OMEGA_2_R_RES 7
#define OMEGA_2_R    BFP_OF_REAL((OMEGA_R*OMEGA_R), OMEGA_2_R_RES)




#define BOOZ_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_UPDATE() {		\
									\
    /* dumb integrate reference attitude        */			\
    const struct Int32Eulers d_angle = {				\
      booz_stabilization_rate_ref.x >> ( F_UPDATE_RES + RATE_REF_RES - ANGLE_REF_RES), \
      booz_stabilization_rate_ref.y >> ( F_UPDATE_RES + RATE_REF_RES - ANGLE_REF_RES), \
      booz_stabilization_rate_ref.z >> ( F_UPDATE_RES + RATE_REF_RES - ANGLE_REF_RES)};	\
    EULERS_ADD(booz_stabilization_att_ref, d_angle );			\
    ANGLE_REF_NORMALIZE(booz_stabilization_att_ref.psi);		\
									\
    /* integrate reference rotational speeds   */			\
    const struct Int32Vect3 d_rate = {					\
      booz_stabilization_accel_ref.x >> ( F_UPDATE_RES + ACCEL_REF_RES - RATE_REF_RES), \
      booz_stabilization_accel_ref.y >> ( F_UPDATE_RES + ACCEL_REF_RES - RATE_REF_RES), \
      booz_stabilization_accel_ref.z >> ( F_UPDATE_RES + ACCEL_REF_RES - RATE_REF_RES)}; \
    VECT3_ADD(booz_stabilization_rate_ref, d_rate);			\
    									\
    /* compute reference attitude error        */			\
    struct Int32Eulers ref_err;						\
    EULERS_DIFF(ref_err, booz_stabilization_att_ref, booz_stabilization_att_sp); \
    /* wrap it in the shortest direction       */			\
    ANGLE_REF_NORMALIZE(ref_err.psi);					\
    									\
    /* compute reference angular accelerations */			\
    const struct Int32Vect3 accel_rate = {				\
      ((int32_t)(-2.*ZETA_OMEGA_P) * (booz_stabilization_rate_ref.x >> (RATE_REF_RES - ACCEL_REF_RES))) \
      >> (ZETA_OMEGA_P_RES),						\
      ((int32_t)(-2.*ZETA_OMEGA_Q) * (booz_stabilization_rate_ref.y >> (RATE_REF_RES - ACCEL_REF_RES))) \
      >> (ZETA_OMEGA_Q_RES),						\
      ((int32_t)(-2.*ZETA_OMEGA_R) * (booz_stabilization_rate_ref.z >> (RATE_REF_RES - ACCEL_REF_RES))) \
      >> (ZETA_OMEGA_R_RES) };						\
    									\
    const struct Int32Vect3 accel_angle = {				\
      ((int32_t)(-OMEGA_2_P)* (ref_err.phi   >> (ANGLE_REF_RES - ACCEL_REF_RES))) >> (OMEGA_2_P_RES), \
      ((int32_t)(-OMEGA_2_Q)* (ref_err.theta >> (ANGLE_REF_RES - ACCEL_REF_RES))) >> (OMEGA_2_Q_RES), \
      ((int32_t)(-OMEGA_2_R)* (ref_err.psi   >> (ANGLE_REF_RES - ACCEL_REF_RES))) >> (OMEGA_2_R_RES) }; \
    									\
    VECT3_SUM(booz_stabilization_accel_ref, accel_rate, accel_angle);	\
									\
    /*	saturate acceleration */					\
    const struct Int32Vect3 MIN_ACCEL = { -ACCEL_REF_MAX_P, -ACCEL_REF_MAX_Q, -ACCEL_REF_MAX_R }; \
    const struct Int32Vect3 MAX_ACCEL = {  ACCEL_REF_MAX_P,  ACCEL_REF_MAX_Q,  ACCEL_REF_MAX_R }; \
    VECT3_BOUND_BOX(booz_stabilization_accel_ref, MIN_ACCEL, MAX_ACCEL); \
    									\
    /* saturate speed and trim accel accordingly */			\
    if (booz_stabilization_rate_ref.x >= RATE_REF_MAX_P) {		\
      booz_stabilization_rate_ref.x = RATE_REF_MAX_P;			\
      if (booz_stabilization_accel_ref.x > 0)				\
	booz_stabilization_accel_ref.x = 0;				\
    }									\
    else if (booz_stabilization_rate_ref.x <= -RATE_REF_MAX_P) {	\
      booz_stabilization_rate_ref.x = -RATE_REF_MAX_P;			\
      if (booz_stabilization_accel_ref.x < 0)				\
	booz_stabilization_accel_ref.x = 0;				\
    }									\
    if (booz_stabilization_rate_ref.y >= RATE_REF_MAX_Q) {		\
      booz_stabilization_rate_ref.y = RATE_REF_MAX_Q;			\
      if (booz_stabilization_accel_ref.y > 0)				\
	booz_stabilization_accel_ref.y = 0;				\
    }									\
    else if (booz_stabilization_rate_ref.y <= -RATE_REF_MAX_Q) {	\
      booz_stabilization_rate_ref.y = -RATE_REF_MAX_Q;			\
      if (booz_stabilization_accel_ref.y < 0)				\
	booz_stabilization_accel_ref.y = 0;				\
    }									\
    if (booz_stabilization_rate_ref.z >= RATE_REF_MAX_R) {		\
      booz_stabilization_rate_ref.z = RATE_REF_MAX_R;			\
      if (booz_stabilization_accel_ref.z > 0)				\
	booz_stabilization_accel_ref.z = 0;				\
    }									\
    else if (booz_stabilization_rate_ref.z <= -RATE_REF_MAX_R) {	\
      booz_stabilization_rate_ref.z = -RATE_REF_MAX_R;			\
      if (booz_stabilization_accel_ref.z < 0)				\
	booz_stabilization_accel_ref.z = 0;				\
    }									\
									\
  }



#define RC_UPDATE_FREQ 40

#define YAW_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_CONTROL_YAW] >  BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_CONTROL_YAW] < -BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R)

#define BOOZ_STABILIZATION_ATTITUDE_READ_RC(_sp, _inflight) {		\
    									\
    _sp.phi =								\
      ((int32_t)-radio_control.values[RADIO_CONTROL_ROLL]  * BOOZ_STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ) \
      << (ANGLE_REF_RES - INT32_ANGLE_FRAC);					\
    _sp.theta =								\
      ((int32_t) radio_control.values[RADIO_CONTROL_PITCH] * BOOZ_STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ) \
      << (ANGLE_REF_RES - INT32_ANGLE_FRAC);					\
    if (_inflight) {							\
      if (YAW_DEADBAND_EXCEEDED()) {					\
	_sp.psi +=							\
	  ((int32_t)-radio_control.values[RADIO_CONTROL_YAW] * BOOZ_STABILIZATION_ATTITUDE_SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ) \
	  << (ANGLE_REF_RES - INT32_ANGLE_FRAC);				\
	ANGLE_REF_NORMALIZE(_sp.psi);					\
      }									\
    }									\
    else { /* if not flying, use current yaw as setpoint */		\
      _sp.psi = (booz_ahrs.ltp_to_body_euler.psi << (ANGLE_REF_RES - INT32_ANGLE_FRAC));		\
    }									\
  }

#define BOOZ_STABILIZATION_ATTITUDE_ADD_SP(_add_sp) {	\
    EULERS_ADD(booz_stabilization_att_sp,_add_sp);	\
    ANGLE_REF_NORMALIZE(booz_stabilization_att_sp.psi); \
}


#define BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp) {		\
    _sp.psi = booz_ahrs.ltp_to_body_euler.psi << (ANGLE_REF_RES - INT32_ANGLE_FRAC); \
    booz_stabilization_att_ref.psi = _sp.psi;				\
    booz_stabilization_rate_ref.z = 0;					\
  }


#endif /* BOOZ2_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_H */
