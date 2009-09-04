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
#ifndef BOOZ_STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H
#define BOOZ_STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H

#include "booz_stabilization.h"

#include "booz_radio_control.h"
#include "math/pprz_algebra_float.h"

#include "stabilization/booz_stabilization_attitude_ref_float.h"

#define RC_UPDATE_FREQ 40.
#define ROLL_COEF  (-BOOZ_STABILIZATION_ATTITUDE_SP_MAX_PHI   / MAX_PPRZ)
#define PITCH_COEF ( BOOZ_STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ)
#define YAW_COEF   (-BOOZ_STABILIZATION_ATTITUDE_SP_MAX_R     / MAX_PPRZ / RC_UPDATE_FREQ)

#define YAW_DEADBAND_EXCEEDED()						\
  (radio_control.values[RADIO_CONTROL_YAW] >  BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_CONTROL_YAW] < -BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R)


#define BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp) {		\
    _sp.psi = booz_ahrs_float.ltp_to_body_euler.psi;	\
    booz_stab_att_ref_euler.psi = _sp.psi;				\
    booz_stab_att_ref_rate.r = 0;					\
    booz_stab_att_ref_accel.r = 0;					\
    struct FloatRMat   sp_rmat;						\
    FLOAT_RMAT_OF_EULERS_312(sp_rmat, _sp);				\
    /*    FLOAT_RMAT_OF_EULERS_321(sp_rmat, _sp);*/			\
    FLOAT_QUAT_OF_RMAT(booz_stab_att_sp_quat, sp_rmat);			\
    FLOAT_RMAT_OF_EULERS_312(sp_rmat, booz_stab_att_ref_euler);				\
    FLOAT_QUAT_OF_RMAT(booz_stab_att_ref_quat, sp_rmat);		\
  }


#define BOOZ_STABILIZATION_ATTITUDE_READ_RC(_sp, _inflight) {		\
    									\
    _sp.phi   = radio_control.values[RADIO_CONTROL_ROLL]  * ROLL_COEF;	\
    _sp.theta = radio_control.values[RADIO_CONTROL_PITCH] * PITCH_COEF; \
    if (_inflight) {							\
      if (YAW_DEADBAND_EXCEEDED()) {					\
	_sp.psi += radio_control.values[RADIO_CONTROL_YAW] * YAW_COEF;	\
	FLOAT_ANGLE_NORMALIZE(_sp.psi);					\
      }									\
      struct FloatRMat   sp_rmat;					\
      FLOAT_RMAT_OF_EULERS_312(sp_rmat, _sp);				\
      /*FLOAT_RMAT_OF_EULERS_321(sp_rmat, _sp);*/			\
      FLOAT_QUAT_OF_RMAT(booz_stab_att_sp_quat, sp_rmat);		\
      /*FLOAT_EULERS_OF_QUAT(sp_euler321, sp_quat);*/			\
    }									\
    else { /* if not flying, use current yaw as setpoint */		\
      BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp);			\
      if (YAW_DEADBAND_EXCEEDED()) {					\
	booz_stab_att_ref_rate.r = RC_UPDATE_FREQ*radio_control.values[RADIO_CONTROL_YAW]*YAW_COEF; \
      }									\
    }									\
									\
  }



#endif /* BOOZ_STABILIZATION_ATTITUDE_REF_QUAT_FLOAT_H */


