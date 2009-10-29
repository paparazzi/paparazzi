/*
 * $Id: booz_stabilization_attitude_euler.c 3795 2009-07-24 23:43:02Z poine $
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
#include "booz_ahrs.h"
#include "booz_radio_control.h"

#include "airframe.h"


struct FloatVect3  booz_stabilization_pgain;
struct FloatVect3  booz_stabilization_dgain;
struct FloatVect3  booz_stabilization_ddgain;
struct FloatVect3  booz_stabilization_igain;
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

  FLOAT_EULERS_ZERO( booz_stabilization_att_sum_err );

}


void booz_stabilization_attitude_read_rc(bool_t in_flight) {

  BOOZ_STABILIZATION_ATTITUDE_READ_RC(booz_stab_att_sp_euler, in_flight);

}


void booz_stabilization_attitude_enter(void) {

  BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF(  booz_stab_att_sp_euler );
  FLOAT_EULERS_ZERO( booz_stabilization_att_sum_err );
  
}


#define MAX_SUM_ERR RadOfDeg(56000)

void booz_stabilization_attitude_run(bool_t  in_flight) {

  booz_stabilization_attitude_ref_update(in_flight);

  /* Compute feedforward */
  booz_stabilization_att_ff_cmd[COMMAND_ROLL] = 
    booz_stabilization_ddgain.x * booz_stab_att_ref_accel.p / 32.;
  booz_stabilization_att_ff_cmd[COMMAND_PITCH] = 
    booz_stabilization_ddgain.y * booz_stab_att_ref_accel.q / 32.;
  booz_stabilization_att_ff_cmd[COMMAND_YAW] = 
    booz_stabilization_ddgain.z * booz_stab_att_ref_accel.r / 32.;

  /* Compute feedback                  */
  /* attitude error            */
  struct FloatEulers att_float;
  EULERS_FLOAT_OF_BFP(att_float, booz_ahrs.ltp_to_body_euler);
  struct FloatEulers att_err;
  EULERS_DIFF(att_err, att_float, booz_stab_att_ref_euler);
  FLOAT_ANGLE_NORMALIZE(att_err.psi);

  if (in_flight) {
    /* update integrator */
    EULERS_ADD(booz_stabilization_att_sum_err, att_err);
    EULERS_BOUND_CUBE(booz_stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  }
  else {
    FLOAT_EULERS_ZERO(booz_stabilization_att_sum_err);
  }
  
  /*  rate error                */
  struct FloatRates rate_float;
  RATES_FLOAT_OF_BFP(rate_float, booz_ahrs.body_rate);
  struct FloatRates rate_err;
  RATES_DIFF(rate_err, rate_float, booz_stab_att_ref_rate);

  /*  PID                  */

  booz_stabilization_att_fb_cmd[COMMAND_ROLL] = 
    booz_stabilization_pgain.x  * att_err.phi +
    booz_stabilization_dgain.x  * rate_err.p +
    booz_stabilization_igain.x  * booz_stabilization_att_sum_err.phi / 1024.;

  booz_stabilization_att_fb_cmd[COMMAND_PITCH] = 
    booz_stabilization_pgain.y  * att_err.theta +
    booz_stabilization_dgain.y  * rate_err.q +
    booz_stabilization_igain.y  * booz_stabilization_att_sum_err.theta / 1024.;

  booz_stabilization_att_fb_cmd[COMMAND_YAW] = 
    booz_stabilization_pgain.z  * att_err.psi +
    booz_stabilization_dgain.z  * rate_err.r +
    booz_stabilization_igain.z  * booz_stabilization_att_sum_err.psi / 1024.;


  booz_stabilization_cmd[COMMAND_ROLL] = 
    (booz_stabilization_att_fb_cmd[COMMAND_ROLL]+booz_stabilization_att_ff_cmd[COMMAND_ROLL])/16.;
  booz_stabilization_cmd[COMMAND_PITCH] = 
    (booz_stabilization_att_fb_cmd[COMMAND_PITCH]+booz_stabilization_att_ff_cmd[COMMAND_PITCH])/16.;
  booz_stabilization_cmd[COMMAND_YAW] = 
    (booz_stabilization_att_fb_cmd[COMMAND_YAW]+booz_stabilization_att_ff_cmd[COMMAND_YAW])/16.;
    
}


