/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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
 *
 */

#include "booz_control.h"

#include "booz_estimator.h"
#include "booz_autopilot.h"
#include "radio_control.h"

#define BOOZ_CONTROL_MIN_THROTTLE 0.05

float booz_control_p_sp;
float booz_control_q_sp;
float booz_control_r_sp;
float booz_control_power_sp;

float booz_control_rate_pq_pgain;
float booz_control_rate_pq_dgain;
float booz_control_rate_r_pgain;
float booz_control_rate_r_dgain;
float booz_control_rate_last_err_p;
float booz_control_rate_last_err_q;
float booz_control_rate_last_err_r;

pprz_t booz_control_commands[COMMANDS_NB];

#define BOOZ_CONTROL_RATE_PQ_PGAIN -700.
#define BOOZ_CONTROL_RATE_PQ_DGAIN   15.

#define BOOZ_CONTROL_RATE_R_PGAIN -600.
#define BOOZ_CONTROL_RATE_R_DGAIN    5.

/* setpoints for max stick throw in degres per second */
#define BOOZ_CONTROL_RATE_PQ_MAX_SP 120.
#define BOOZ_CONTROL_RATE_R_MAX_SP  100.


float booz_control_attitude_phi_sp;
float booz_control_attitude_theta_sp;
float booz_control_attitude_psi_sp;
float booz_control_attitude_phi_theta_pgain;
float booz_control_attitude_phi_theta_dgain;
float booz_control_attitude_psi_pgain;
float booz_control_attitude_psi_dgain;

#define BOOZ_CONTROL_ATTITUDE_PHI_THETA_PGAIN  -1250.
#define BOOZ_CONTROL_ATTITUDE_PHI_THETA_DGAIN   -700.

#define BOOZ_CONTROL_ATTITUDE_PSI_PGAIN  -1050.
#define BOOZ_CONTROL_ATTITUDE_PSI_DGAIN   -850.

/* setpoints for max stick throw in degres */
#define BOOZ_CONTROL_ATTITUDE_PHI_THETA_MAX_SP 30.
#define BOOZ_CONTROL_ATTITUDE_PSI_MAX_SP 45.

#define BOOZ_CONTROL_ATTITUDE_DT_UPDATE_SP (1./50.)

void booz_control_init(void) {

  booz_control_p_sp = 0.;
  booz_control_q_sp = 0.;
  booz_control_r_sp = 0.;
  booz_control_power_sp = 0.;

  booz_control_rate_last_err_p = 0.;
  booz_control_rate_last_err_q = 0.;
  booz_control_rate_last_err_r = 0.;

  booz_control_rate_pq_pgain = BOOZ_CONTROL_RATE_PQ_PGAIN;
  booz_control_rate_pq_dgain = BOOZ_CONTROL_RATE_PQ_DGAIN;
  booz_control_rate_r_pgain = BOOZ_CONTROL_RATE_R_PGAIN;
  booz_control_rate_r_dgain = BOOZ_CONTROL_RATE_R_DGAIN;


  booz_control_attitude_phi_sp = 0.;
  booz_control_attitude_theta_sp =0.;
  booz_control_attitude_psi_sp =0.;
  booz_control_attitude_phi_theta_pgain = BOOZ_CONTROL_ATTITUDE_PHI_THETA_PGAIN;
  booz_control_attitude_phi_theta_dgain = BOOZ_CONTROL_ATTITUDE_PHI_THETA_DGAIN;
  booz_control_attitude_psi_pgain = BOOZ_CONTROL_ATTITUDE_PSI_PGAIN;
  booz_control_attitude_psi_dgain = BOOZ_CONTROL_ATTITUDE_PSI_DGAIN;

}


void booz_control_rate_read_setpoints_from_rc(void) {

  booz_control_p_sp = -rc_values[RADIO_ROLL]  * RadOfDeg(BOOZ_CONTROL_RATE_PQ_MAX_SP)/MAX_PPRZ;
  booz_control_q_sp =  rc_values[RADIO_PITCH] * RadOfDeg(BOOZ_CONTROL_RATE_PQ_MAX_SP)/MAX_PPRZ;
  booz_control_r_sp = -rc_values[RADIO_YAW]   * RadOfDeg(BOOZ_CONTROL_RATE_R_MAX_SP)/MAX_PPRZ;
  booz_control_power_sp = rc_values[RADIO_THROTTLE] / (float)MAX_PPRZ;

}


void booz_control_rate_run(void) {

  if (booz_control_power_sp < BOOZ_CONTROL_MIN_THROTTLE) {
    booz_control_commands[COMMAND_P] = 0;
    booz_control_commands[COMMAND_Q] = 0;
    booz_control_commands[COMMAND_R] = 0;
    booz_control_commands[COMMAND_THROTTLE] = 0;
  }
  else {
    const float rate_err_p = booz_estimator_uf_p - booz_control_p_sp;
    const float rate_d_err_p = rate_err_p - booz_control_rate_last_err_p;
    booz_control_rate_last_err_p = rate_err_p;
    const float cmd_p = booz_control_rate_pq_pgain * ( rate_err_p + booz_control_rate_pq_dgain * rate_d_err_p );
    
    const float rate_err_q = booz_estimator_uf_q - booz_control_q_sp;
    const float rate_d_err_q = rate_err_q - booz_control_rate_last_err_q;
    booz_control_rate_last_err_q = rate_err_q;
    const float cmd_q = booz_control_rate_pq_pgain * ( rate_err_q + booz_control_rate_pq_dgain * rate_d_err_q );
    
    const float rate_err_r = booz_estimator_uf_r - booz_control_r_sp;
    const float rate_d_err_r = rate_err_r - booz_control_rate_last_err_r;
    booz_control_rate_last_err_r = rate_err_r;
    const float cmd_r = booz_control_rate_r_pgain * ( rate_err_r + booz_control_rate_r_dgain * rate_d_err_r );

    booz_control_commands[COMMAND_P] = TRIM_PPRZ((int16_t)cmd_p);
    booz_control_commands[COMMAND_Q] = TRIM_PPRZ((int16_t)cmd_q);
    booz_control_commands[COMMAND_R] = TRIM_PPRZ((int16_t)cmd_r);
    booz_control_commands[COMMAND_THROTTLE] = TRIM_PPRZ((int16_t) (booz_control_power_sp * MAX_PPRZ));
  }

}

void booz_control_attitude_read_setpoints_from_rc(void) {

  booz_control_attitude_phi_sp = -rc_values[RADIO_ROLL]  * 
                                  RadOfDeg(BOOZ_CONTROL_ATTITUDE_PHI_THETA_MAX_SP)/MAX_PPRZ;
  booz_control_attitude_theta_sp =  rc_values[RADIO_PITCH] * 
                                  RadOfDeg(BOOZ_CONTROL_ATTITUDE_PHI_THETA_MAX_SP)/MAX_PPRZ;
  if (booz_autopilot_mode == BOOZ_AP_MODE_ATTITUDE) {
    booz_control_r_sp = -rc_values[RADIO_YAW]   * RadOfDeg(BOOZ_CONTROL_RATE_R_MAX_SP)/MAX_PPRZ;
  }
  else { /* BOOZ_AP_MODE_HEADING_HOLD */
    if (booz_control_power_sp < BOOZ_CONTROL_MIN_THROTTLE)
      booz_control_attitude_psi_sp = booz_estimator_psi;
    else {
      booz_control_attitude_psi_sp +=  -rc_values[RADIO_YAW] * 
	RadOfDeg(BOOZ_CONTROL_ATTITUDE_PSI_MAX_SP)*BOOZ_CONTROL_ATTITUDE_DT_UPDATE_SP/MAX_PPRZ;
    }
  }
  booz_control_power_sp = rc_values[RADIO_THROTTLE] / (float)MAX_PPRZ;
}

void booz_control_attitude_run(void) {

  if (booz_control_power_sp < BOOZ_CONTROL_MIN_THROTTLE) {
    booz_control_commands[COMMAND_P] = 0;
    booz_control_commands[COMMAND_Q] = 0;
    booz_control_commands[COMMAND_R] = 0;
    booz_control_commands[COMMAND_THROTTLE] = 0;
  }
  else {
    const float att_err_phi = booz_estimator_phi - booz_control_attitude_phi_sp;
    const float cmd_p = booz_control_attitude_phi_theta_pgain *  att_err_phi + 
      booz_control_attitude_phi_theta_dgain * booz_estimator_p ;
    
    const float att_err_theta = booz_estimator_theta - booz_control_attitude_theta_sp;
    const float cmd_q = booz_control_attitude_phi_theta_pgain * att_err_theta + 
      booz_control_attitude_phi_theta_dgain * booz_estimator_q;
    
    float cmd_r;
    if (booz_autopilot_mode == BOOZ_AP_MODE_ATTITUDE) {
      const float rate_err_r = booz_estimator_r - booz_control_r_sp;
      const float rate_d_err_r = rate_err_r - booz_control_rate_last_err_r;
      booz_control_rate_last_err_r = rate_err_r;
      cmd_r = booz_control_rate_r_pgain * ( rate_err_r + booz_control_rate_r_dgain * rate_d_err_r );
    }
    else {
      float att_err_psi = booz_estimator_psi - booz_control_attitude_psi_sp;
      NormRadAngle(att_err_psi);
      cmd_r = booz_control_attitude_psi_pgain * att_err_psi + 
	booz_control_attitude_psi_dgain * booz_estimator_r;
    }

    booz_control_commands[COMMAND_P] = TRIM_PPRZ((int16_t)cmd_p);
    booz_control_commands[COMMAND_Q] = TRIM_PPRZ((int16_t)cmd_q);
    booz_control_commands[COMMAND_R] = TRIM_PPRZ((int16_t)cmd_r);
    booz_control_commands[COMMAND_THROTTLE] = TRIM_PPRZ((int16_t) (booz_control_power_sp * MAX_PPRZ));
  }
}
