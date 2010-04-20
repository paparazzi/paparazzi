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

#include "booz_stabilization.h"
#include "booz_ahrs.h"
#include "booz_radio_control.h"



#include "airframe.h"

struct Int32AttitudeGains  booz_stabilization_gains;

struct Int32Eulers booz_stabilization_att_sum_err;

int32_t booz_stabilization_att_fb_cmd[COMMANDS_NB];
int32_t booz_stabilization_att_ff_cmd[COMMANDS_NB];

void booz_stabilization_attitude_init(void) {

  booz_stabilization_attitude_ref_init();
  
 
  VECT3_ASSIGN(booz_stabilization_gains.p,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_PGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_PGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_PGAIN);
  
  VECT3_ASSIGN(booz_stabilization_gains.d,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_DGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_DGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_DGAIN);
  
  VECT3_ASSIGN(booz_stabilization_gains.i,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_IGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_IGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_IGAIN);

  VECT3_ASSIGN(booz_stabilization_gains.dd,
	       BOOZ_STABILIZATION_ATTITUDE_PHI_DDGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_THETA_DDGAIN,
	       BOOZ_STABILIZATION_ATTITUDE_PSI_DDGAIN);
  

  INT_EULERS_ZERO( booz_stabilization_att_sum_err );

}


void booz_stabilization_attitude_read_rc(bool_t in_flight) {

  BOOZ_STABILIZATION_ATTITUDE_READ_RC(booz_stab_att_sp_euler, in_flight);

}


void booz_stabilization_attitude_enter(void) {

  BOOZ_STABILIZATION_ATTITUDE_RESET_PSI_REF(  booz_stab_att_sp_euler );
  INT_EULERS_ZERO( booz_stabilization_att_sum_err );
  
}


#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b)) 
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

#define MAX_SUM_ERR 4000000

void booz_stabilization_attitude_run(bool_t  in_flight) {


  /* update reference */
  booz_stabilization_attitude_ref_update();

  /* compute feedforward command */
  booz_stabilization_att_ff_cmd[COMMAND_ROLL] = 
    OFFSET_AND_ROUND(booz_stabilization_gains.dd.x * booz_stab_att_ref_accel.p, 5);
  booz_stabilization_att_ff_cmd[COMMAND_PITCH] = 
    OFFSET_AND_ROUND(booz_stabilization_gains.dd.y * booz_stab_att_ref_accel.q, 5);
  booz_stabilization_att_ff_cmd[COMMAND_YAW] = 
    OFFSET_AND_ROUND(booz_stabilization_gains.dd.z * booz_stab_att_ref_accel.r, 5);

  /* compute feedback command */
  /* attitude error            */
  const struct Int32Eulers att_ref_scaled = {
    OFFSET_AND_ROUND(booz_stab_att_ref_euler.phi,   (REF_ANGLE_FRAC - INT32_ANGLE_FRAC)),
    OFFSET_AND_ROUND(booz_stab_att_ref_euler.theta, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC)),
    OFFSET_AND_ROUND(booz_stab_att_ref_euler.psi,   (REF_ANGLE_FRAC - INT32_ANGLE_FRAC)) };
  struct Int32Eulers att_err;
  EULERS_DIFF(att_err, booz_ahrs.ltp_to_body_euler, att_ref_scaled);
  INT32_ANGLE_NORMALIZE(att_err.psi);

  if (in_flight) {
    /* update integrator */
    EULERS_ADD(booz_stabilization_att_sum_err, att_err);
    EULERS_BOUND_CUBE(booz_stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  }
  else {
    INT_EULERS_ZERO(booz_stabilization_att_sum_err);
  }
  
  /* rate error                */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(booz_stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(booz_stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(booz_stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC)) };
  struct Int32Rates rate_err;
  RATES_DIFF(rate_err, booz_ahrs.body_rate, rate_ref_scaled);

  /* PID                  */
  booz_stabilization_att_fb_cmd[COMMAND_ROLL] = 
    booz_stabilization_gains.p.x    * att_err.phi +
    booz_stabilization_gains.d.x    * rate_err.p +
    OFFSET_AND_ROUND2((booz_stabilization_gains.i.x  * booz_stabilization_att_sum_err.phi), 10);

  booz_stabilization_att_fb_cmd[COMMAND_PITCH] = 
    booz_stabilization_gains.p.y    * att_err.theta +
    booz_stabilization_gains.d.y    * rate_err.q +
    OFFSET_AND_ROUND2((booz_stabilization_gains.i.y  * booz_stabilization_att_sum_err.theta), 10);

  booz_stabilization_att_fb_cmd[COMMAND_YAW] = 
    booz_stabilization_gains.p.z    * att_err.psi +
    booz_stabilization_gains.d.z    * rate_err.r +
    OFFSET_AND_ROUND2((booz_stabilization_gains.i.z  * booz_stabilization_att_sum_err.psi), 10);
    
  /* sum feedforward and feedback */
  booz_stabilization_cmd[COMMAND_ROLL] = 
    OFFSET_AND_ROUND((booz_stabilization_att_fb_cmd[COMMAND_ROLL]+booz_stabilization_att_ff_cmd[COMMAND_ROLL]), 16);
  
  booz_stabilization_cmd[COMMAND_PITCH] = 
    OFFSET_AND_ROUND((booz_stabilization_att_fb_cmd[COMMAND_PITCH]+booz_stabilization_att_ff_cmd[COMMAND_PITCH]), 16);
  
  booz_stabilization_cmd[COMMAND_YAW] = 
    OFFSET_AND_ROUND((booz_stabilization_att_fb_cmd[COMMAND_YAW]+booz_stabilization_att_ff_cmd[COMMAND_YAW]), 16);
  
}


