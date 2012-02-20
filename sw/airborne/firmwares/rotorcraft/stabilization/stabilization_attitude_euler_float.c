/*
 * $Id: stabilization_attitude_euler.c 3795 2009-07-24 23:43:02Z poine $
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

#include "firmwares/rotorcraft/stabilization.h"

#include "math/pprz_algebra_float.h"
#include "subsystems/ahrs.h"
#include "subsystems/radio_control.h"

#include "generated/airframe.h"


struct FloatAttitudeGains stabilization_gains;

/* warn if some gains are still negative */
#if (STABILIZATION_ATTITUDE_FLOAT_PHI_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_FLOAT_THETA_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_FLOAT_PSI_PGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_FLOAT_PHI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_FLOAT_THETA_DGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_FLOAT_PSI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_FLOAT_PHI_IGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_FLOAT_THETA_IGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_FLOAT_PSI_IGAIN  < 0)
#warning "ALL control gains are now positive!!!"
#endif

struct FloatEulers stabilization_att_sum_err;

float stabilization_att_fb_cmd[COMMANDS_NB];
float stabilization_att_ff_cmd[COMMANDS_NB];


void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();

  VECT3_ASSIGN(stabilization_gains.p,
               STABILIZATION_ATTITUDE_FLOAT_PHI_PGAIN,
               STABILIZATION_ATTITUDE_FLOAT_THETA_PGAIN,
               STABILIZATION_ATTITUDE_FLOAT_PSI_PGAIN);

  VECT3_ASSIGN(stabilization_gains.d,
               STABILIZATION_ATTITUDE_FLOAT_PHI_DGAIN,
               STABILIZATION_ATTITUDE_FLOAT_THETA_DGAIN,
               STABILIZATION_ATTITUDE_FLOAT_PSI_DGAIN);

  VECT3_ASSIGN(stabilization_gains.i,
               STABILIZATION_ATTITUDE_FLOAT_PHI_IGAIN,
               STABILIZATION_ATTITUDE_FLOAT_THETA_IGAIN,
               STABILIZATION_ATTITUDE_FLOAT_PSI_IGAIN);

  VECT3_ASSIGN(stabilization_gains.dd,
               STABILIZATION_ATTITUDE_FLOAT_PHI_DDGAIN,
               STABILIZATION_ATTITUDE_FLOAT_THETA_DDGAIN,
               STABILIZATION_ATTITUDE_FLOAT_PSI_DDGAIN);

  FLOAT_EULERS_ZERO( stabilization_att_sum_err );

}


void stabilization_attitude_read_rc(bool_t in_flight) {

  STABILIZATION_ATTITUDE_FLOAT_READ_RC(stab_att_sp_euler, in_flight);

}


void stabilization_attitude_enter(void) {

  STABILIZATION_ATTITUDE_FLOAT_RESET_PSI_REF(  stab_att_sp_euler );
  FLOAT_EULERS_ZERO( stabilization_att_sum_err );

}


#define MAX_SUM_ERR RadOfDeg(56000)

void stabilization_attitude_run(bool_t  in_flight) {

  stabilization_attitude_ref_update();

  /* Compute feedforward */
  stabilization_att_ff_cmd[COMMAND_ROLL] =
    stabilization_gains.dd.x * stab_att_ref_accel.p / 32.;
  stabilization_att_ff_cmd[COMMAND_PITCH] =
    stabilization_gains.dd.y * stab_att_ref_accel.q / 32.;
  stabilization_att_ff_cmd[COMMAND_YAW] =
    stabilization_gains.dd.z * stab_att_ref_accel.r / 32.;

  /* Compute feedback                  */
  /* attitude error            */
  struct FloatEulers att_float;
  EULERS_FLOAT_OF_BFP(att_float, ahrs.ltp_to_body_euler);
  struct FloatEulers att_err;
  EULERS_DIFF(att_err, stab_att_ref_euler, att_float);
  FLOAT_ANGLE_NORMALIZE(att_err.psi);

  if (in_flight) {
    /* update integrator */
    EULERS_ADD(stabilization_att_sum_err, att_err);
    EULERS_BOUND_CUBE(stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  }
  else {
    FLOAT_EULERS_ZERO(stabilization_att_sum_err);
  }

  /*  rate error                */
  struct FloatRates rate_float;
  RATES_FLOAT_OF_BFP(rate_float, ahrs.body_rate);
  struct FloatRates rate_err;
  RATES_DIFF(rate_err, stab_att_ref_rate, rate_float);

  /*  PID                  */

  stabilization_att_fb_cmd[COMMAND_ROLL] =
    stabilization_gains.p.x  * att_err.phi +
    stabilization_gains.d.x  * rate_err.p +
    stabilization_gains.i.x  * stabilization_att_sum_err.phi / 1024.;

  stabilization_att_fb_cmd[COMMAND_PITCH] =
    stabilization_gains.p.y  * att_err.theta +
    stabilization_gains.d.y  * rate_err.q +
    stabilization_gains.i.y  * stabilization_att_sum_err.theta / 1024.;

  stabilization_att_fb_cmd[COMMAND_YAW] =
    stabilization_gains.p.z  * att_err.psi +
    stabilization_gains.d.z  * rate_err.r +
    stabilization_gains.i.z  * stabilization_att_sum_err.psi / 1024.;


  stabilization_cmd[COMMAND_ROLL] =
    (stabilization_att_fb_cmd[COMMAND_ROLL]+stabilization_att_ff_cmd[COMMAND_ROLL])/16.;
  stabilization_cmd[COMMAND_PITCH] =
    (stabilization_att_fb_cmd[COMMAND_PITCH]+stabilization_att_ff_cmd[COMMAND_PITCH])/16.;
  stabilization_cmd[COMMAND_YAW] =
    (stabilization_att_fb_cmd[COMMAND_YAW]+stabilization_att_ff_cmd[COMMAND_YAW])/16.;

}
