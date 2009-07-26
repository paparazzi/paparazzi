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

#ifndef BOOZ_STABILIZATION_ATTITUDE_REF_EULER_INT_H
#define BOOZ_STABILIZATION_ATTITUDE_REF_EULER_INT_H

#include "stabilization/booz_stabilization_attitude_ref_int.h"

#include "booz_radio_control.h"


#define REF_ACCEL_FRAC 12
#define REF_RATE_FRAC  16
#define REF_ANGLE_FRAC 20
#define REF_ANGLE_PI      BFP_OF_REAL(3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define REF_ANGLE_TWO_PI  BFP_OF_REAL(2.*3.1415926535897932384626433832795029, REF_ANGLE_FRAC)
#define ANGLE_REF_NORMALIZE(_a) {					\
    while (_a >  REF_ANGLE_PI)  _a -= REF_ANGLE_TWO_PI;			\
    while (_a < -REF_ANGLE_PI)  _a += REF_ANGLE_TWO_PI;			\
  }



/*
 * Radio Control
 */
#define SP_MAX_PHI   ANGLE_BFP_OF_REAL(BOOZ_STABILIZATION_ATTITUDE_SP_MAX_PHI)
#define SP_MAX_THETA ANGLE_BFP_OF_REAL(BOOZ_STABILIZATION_ATTITUDE_SP_MAX_THETA)
#define SP_MAX_R     ANGLE_BFP_OF_REAL(BOOZ_STABILIZATION_ATTITUDE_SP_MAX_R)





#define RC_UPDATE_FREQ 40

#define YAW_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_CONTROL_YAW] >  BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_CONTROL_YAW] < -BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R)

#define BOOZ_STABILIZATION_ATTITUDE_READ_RC(_sp, _inflight) {		\
    									\
    _sp.phi =								\
      ((int32_t)-radio_control.values[RADIO_CONTROL_ROLL]  * (int32_t)SP_MAX_PHI / MAX_PPRZ) \
      << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);					\
    _sp.theta =								\
      ((int32_t) radio_control.values[RADIO_CONTROL_PITCH] * (int32_t)SP_MAX_THETA / MAX_PPRZ) \
      << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);					\
    if (_inflight) {							\
      if (YAW_DEADBAND_EXCEEDED()) {					\
	_sp.psi +=							\
	  ((int32_t)-radio_control.values[RADIO_CONTROL_YAW] * (int32_t)SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ) \
	  << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);				\
	ANGLE_REF_NORMALIZE(_sp.psi);					\
      }									\
    }									\
    else { /* if not flying, use current yaw as setpoint */		\
      _sp.psi = (booz_ahrs.ltp_to_body_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));		\
    }									\
  }

#define BOOZ_STABILIZATION_ATTITUDE_ADD_SP(_add_sp) {	\
    EULERS_ADD(booz_stab_att_sp_euler,_add_sp);	\
    ANGLE_REF_NORMALIZE(booz_stab_att_sp_euler.psi); \
}

#define BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp) {		\
    _sp.psi = booz_ahrs.ltp_to_body_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC); \
    booz_stab_att_ref_euler.psi = _sp.psi;				\
    booz_stab_att_ref_rate.r = 0;					\
  }


#endif /* BOOZ2_STABILIZATION_ATTITUDE_REF_EULER_INT_H */
