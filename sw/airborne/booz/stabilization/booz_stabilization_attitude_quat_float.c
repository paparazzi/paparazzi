/*
 * $Id: booz_stabilization_attitude_euler.c 3787 2009-07-24 15:33:54Z poine $
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

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "booz_ahrs.h"
#include "booz_radio_control.h"

#include "airframe.h"


struct FloatVect3  booz_stabilization_pgain;
struct FloatVect3  booz_stabilization_dgain;
struct FloatVect3  booz_stabilization_ddgain;
struct FloatVect3  booz_stabilization_igain;
struct FloatQuat   booz_stabilization_sum_err;
struct FloatEulers booz_stabilization_att_sum_err;

float booz_stabilization_att_fb_cmd[COMMANDS_NB];
float booz_stabilization_att_ff_cmd[COMMANDS_NB];


void booz_stabilization_attitude_init(void) {

  booz_stabilization_attitude_ref_init();

  VECT3_ASSIGN(booz_stabilization_pgain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_PGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_PGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_PGAIN);
  
  VECT3_ASSIGN(booz_stabilization_dgain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_DGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_DGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_DGAIN);
  
  VECT3_ASSIGN(booz_stabilization_igain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_IGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_IGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_IGAIN);

  VECT3_ASSIGN(booz_stabilization_ddgain,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_DDGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_DDGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_DDGAIN);

  FLOAT_QUAT_ZERO( booz_stabilization_sum_err );
  FLOAT_EULERS_ZERO( booz_stabilization_att_sum_err );

}

static void reset_psi_ref_from_body(void) {
    booz_stab_att_sp_euler.psi = booz_ahrs_float.ltp_to_body_euler.psi;
    booz_stab_att_ref_euler.psi = booz_ahrs_float.ltp_to_body_euler.psi;
    booz_stab_att_ref_rate.r = 0;
    booz_stab_att_ref_accel.r = 0;
}

static void update_sp_quat_from_eulers(void) {
    struct FloatRMat sp_rmat;

    FLOAT_RMAT_OF_EULERS_312(sp_rmat, booz_stab_att_sp_euler);
    /*    FLOAT_RMAT_OF_EULERS_321(sp_rmat, _sp);*/
    FLOAT_QUAT_OF_RMAT(booz_stab_att_sp_quat, sp_rmat);
}

static void update_ref_quat_from_eulers(void) {
    struct FloatRMat ref_rmat;

    FLOAT_RMAT_OF_EULERS_312(ref_rmat, booz_stab_att_ref_euler);
    FLOAT_QUAT_OF_RMAT(booz_stab_att_ref_quat, ref_rmat);
}

void booz_stabilization_attitude_read_beta_vane(float beta)
{
  booz_stab_att_sp_euler.psi += booz_ahrs_float.ltp_to_body_euler.theta * beta / RC_UPDATE_FREQ;
  update_sp_quat_from_eulers();
}

void booz_stabilization_attitude_read_rc(bool_t in_flight) {

    booz_stab_att_sp_euler.phi   = radio_control.values[RADIO_CONTROL_ROLL]  * ROLL_COEF;
    booz_stab_att_sp_euler.theta = radio_control.values[RADIO_CONTROL_PITCH] * PITCH_COEF;

    if (in_flight) {
      if (YAW_DEADBAND_EXCEEDED()) {
        booz_stab_att_sp_euler.psi += radio_control.values[RADIO_CONTROL_YAW] * YAW_COEF;
        FLOAT_ANGLE_NORMALIZE(booz_stab_att_sp_euler.psi);
      }
      update_sp_quat_from_eulers();
    } else { /* if not flying, use current yaw as setpoint */
      reset_psi_ref_from_body();
      update_sp_quat_from_eulers();
      update_ref_quat_from_eulers();
      booz_stab_att_ref_rate.r = RC_UPDATE_FREQ*radio_control.values[RADIO_CONTROL_YAW]*YAW_COEF;
    }

}



void booz_stabilization_attitude_enter(void) {

  reset_psi_ref_from_body();
  update_sp_quat_from_eulers();
  update_ref_quat_from_eulers();
  
  FLOAT_EULERS_ZERO( booz_stabilization_att_sum_err );
  FLOAT_QUAT_ZERO( booz_stabilization_sum_err );
  
}


#define IERROR_SCALE 1024

#define MAX_SUM_ERR RadOfDeg(56000)
#include <stdio.h>
void booz_stabilization_attitude_run(bool_t  in_flight) {
#ifdef BOOZ_AHRS_FIXED_POINT
  BOOZ_AHRS_FLOAT_OF_INT32();
#endif 

  
  /* 
   * Update reference
   */
  booz_stabilization_attitude_ref_update();

  /* 
   * Compute feedforward
   */
  /* ref accel bfp is 2^12 and int controller offsets by 5 */
  booz_stabilization_att_ff_cmd[COMMAND_ROLL] = 
    booz_stabilization_ddgain.x * BFP_OF_REAL(booz_stab_att_ref_accel.p, (12-5));
  booz_stabilization_att_ff_cmd[COMMAND_PITCH] = 
    booz_stabilization_ddgain.y * BFP_OF_REAL(booz_stab_att_ref_accel.q, (12-5));
  booz_stabilization_att_ff_cmd[COMMAND_YAW] = 
    booz_stabilization_ddgain.z * BFP_OF_REAL(booz_stab_att_ref_accel.r, (12-5));

  /*
   * Compute feedback                        
   */

  /* attitude error                          */
  struct FloatQuat att_err; 
  FLOAT_QUAT_INV_COMP(att_err, booz_ahrs_float.ltp_to_body_quat, booz_stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  FLOAT_QUAT_WRAP_SHORTEST(att_err);  

  if (in_flight) {
    struct FloatQuat new_sum_err, scaled_att_err;
    /* update accumulator */
    FLOAT_QUAT_COPY(scaled_att_err, att_err);
    scaled_att_err.qx /= IERROR_SCALE;
    scaled_att_err.qy /= IERROR_SCALE;
    scaled_att_err.qz /= IERROR_SCALE;
    FLOAT_QUAT_COMP_INV(new_sum_err, booz_stabilization_sum_err, scaled_att_err);
    FLOAT_QUAT_NORMALISE(new_sum_err);
    FLOAT_QUAT_COPY(booz_stabilization_sum_err, new_sum_err);
  }
  else {
    /* reset accumulator */
    FLOAT_EULERS_ZERO(booz_stabilization_att_sum_err);
    FLOAT_QUAT_ZERO( booz_stabilization_sum_err );
  }
  
  /*  rate error                */
  struct FloatRates rate_err;
  RATES_DIFF(rate_err, booz_ahrs_float.body_rate, booz_stab_att_ref_rate);

  /*  PID                  */
  booz_stabilization_att_fb_cmd[COMMAND_ROLL] = 
    -2. * booz_stabilization_pgain.x  * QUAT1_BFP_OF_REAL(att_err.qx)+
    booz_stabilization_dgain.x  * RATE_BFP_OF_REAL(rate_err.p) +
    booz_stabilization_igain.x  * QUAT1_BFP_OF_REAL(booz_stabilization_sum_err.qx);

  booz_stabilization_att_fb_cmd[COMMAND_PITCH] = 
    -2. * booz_stabilization_pgain.y  * QUAT1_BFP_OF_REAL(att_err.qy)+
    booz_stabilization_dgain.y  * RATE_BFP_OF_REAL(rate_err.q) +
    booz_stabilization_igain.y  * QUAT1_BFP_OF_REAL(booz_stabilization_sum_err.qy);
  
  booz_stabilization_att_fb_cmd[COMMAND_YAW] = 
    -2. * booz_stabilization_pgain.z  * QUAT1_BFP_OF_REAL(att_err.qz)+
    booz_stabilization_dgain.z  * RATE_BFP_OF_REAL(rate_err.r) +
    booz_stabilization_igain.z  * QUAT1_BFP_OF_REAL(booz_stabilization_sum_err.qz);


  booz_stabilization_cmd[COMMAND_ROLL] = 
    ((booz_stabilization_att_fb_cmd[COMMAND_ROLL]+booz_stabilization_att_ff_cmd[COMMAND_ROLL]))/(1<<16);
  booz_stabilization_cmd[COMMAND_PITCH] = 
    ((booz_stabilization_att_fb_cmd[COMMAND_PITCH]+booz_stabilization_att_ff_cmd[COMMAND_PITCH]))/(1<<16);
  booz_stabilization_cmd[COMMAND_YAW] = 
    ((booz_stabilization_att_fb_cmd[COMMAND_YAW]+booz_stabilization_att_ff_cmd[COMMAND_YAW]))/(1<<16);
    
}



