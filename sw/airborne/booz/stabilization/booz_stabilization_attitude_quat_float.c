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
  /* FIXME : I don't understand the /32 - should be /256 */
  booz_stabilization_att_ff_cmd[COMMAND_ROLL] = 
    booz_stabilization_ddgain.x * booz_stab_att_ref_accel.p / 32.;
  booz_stabilization_att_ff_cmd[COMMAND_PITCH] = 
    booz_stabilization_ddgain.y * booz_stab_att_ref_accel.q / 32.;
  booz_stabilization_att_ff_cmd[COMMAND_YAW] = 
    booz_stabilization_ddgain.z * booz_stab_att_ref_accel.r / 32.;

  /*
   * Compute feedback                        
   */

  /* attitude error                          */


  struct FloatQuat att_err; 
  FLOAT_QUAT_INV_COMP(att_err, booz_ahrs_float.ltp_to_body_quat, booz_stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  FLOAT_QUAT_WRAP_SHORTEST(att_err);  

  if (in_flight) {
    /* update accumulator */
    //    EULERS_ADD(booz_stabilization_att_sum_err, err);
    EULERS_BOUND_CUBE(booz_stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  }
  else {
    /* reset accumulator */
    FLOAT_EULERS_ZERO(booz_stabilization_att_sum_err);
  }
  
  /*  rate error                */

  struct FloatRates rate_err;
  RATES_DIFF(rate_err, booz_ahrs_float.body_rate, booz_stab_att_ref_rate);

  /*  PID                  */

  booz_stabilization_att_fb_cmd[COMMAND_ROLL] = 
    -2. * booz_stabilization_pgain.x  * att_err.qx +
    booz_stabilization_dgain.x  * rate_err.p +
    booz_stabilization_igain.x  * booz_stabilization_att_sum_err.phi / 1024.;

  booz_stabilization_att_fb_cmd[COMMAND_PITCH] = 
    -2. * booz_stabilization_pgain.y  * att_err.qy +
    booz_stabilization_dgain.y  * rate_err.q +
    booz_stabilization_igain.y  * booz_stabilization_att_sum_err.theta / 1024.;

  booz_stabilization_att_fb_cmd[COMMAND_YAW] = 
    -2. * booz_stabilization_pgain.z  * att_err.qz +
    booz_stabilization_dgain.z  * rate_err.r +
    booz_stabilization_igain.z  * booz_stabilization_att_sum_err.psi / 1024.;


  booz_stabilization_cmd[COMMAND_ROLL] = 
    (booz_stabilization_att_fb_cmd[COMMAND_ROLL]+booz_stabilization_att_ff_cmd[COMMAND_ROLL])/16.;
  booz_stabilization_cmd[COMMAND_PITCH] = 
    (booz_stabilization_att_fb_cmd[COMMAND_PITCH]+booz_stabilization_att_ff_cmd[COMMAND_PITCH])/16.;
  booz_stabilization_cmd[COMMAND_YAW] = 
    (booz_stabilization_att_fb_cmd[COMMAND_YAW]+booz_stabilization_att_ff_cmd[COMMAND_YAW])/16.;
    
}



