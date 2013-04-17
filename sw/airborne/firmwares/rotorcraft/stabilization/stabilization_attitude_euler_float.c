/*
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
#include "state.h"
#include "subsystems/radio_control.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include "generated/airframe.h"

#include "math/pprz_algebra_float.h"

struct FloatAttitudeGains stabilization_gains;
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

  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight);

}


void stabilization_attitude_enter(void) {
  stab_att_sp_euler.psi = stateGetNedToBodyEulers_f()->psi;
  reset_psi_ref_from_body();
  FLOAT_EULERS_ZERO(stabilization_att_sum_err);
}


#define MAX_SUM_ERR 200

void stabilization_attitude_run(bool_t  in_flight) {

  stabilization_attitude_ref_update();

  /* Compute feedforward */
  stabilization_att_ff_cmd[COMMAND_ROLL] =
    stabilization_gains.dd.x * stab_att_ref_accel.p;
  stabilization_att_ff_cmd[COMMAND_PITCH] =
    stabilization_gains.dd.y * stab_att_ref_accel.q;
  stabilization_att_ff_cmd[COMMAND_YAW] =
    stabilization_gains.dd.z * stab_att_ref_accel.r;

  /* Compute feedback                  */
  /* attitude error            */
  struct FloatEulers att_err;
  struct FloatEulers* ltp_to_body_euler = stateGetNedToBodyEulers_f();
  EULERS_DIFF(att_err, stab_att_ref_euler, (*ltp_to_body_euler));
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
  struct FloatRates* body_rate = stateGetBodyRates_f();
  struct FloatRates rate_err;
  RATES_DIFF(rate_err, stab_att_ref_rate, (*body_rate));

  /*  PID                  */
  stabilization_att_fb_cmd[COMMAND_ROLL] =
    stabilization_gains.p.x  * att_err.phi +
    stabilization_gains.d.x  * rate_err.p +
    stabilization_gains.i.x  * stabilization_att_sum_err.phi;

  stabilization_att_fb_cmd[COMMAND_PITCH] =
    stabilization_gains.p.y  * att_err.theta +
    stabilization_gains.d.y  * rate_err.q +
    stabilization_gains.i.y  * stabilization_att_sum_err.theta;

  stabilization_att_fb_cmd[COMMAND_YAW] =
    stabilization_gains.p.z  * att_err.psi +
    stabilization_gains.d.z  * rate_err.r +
    stabilization_gains.i.z  * stabilization_att_sum_err.psi;


  stabilization_cmd[COMMAND_ROLL] =
    (int32_t)(stabilization_att_fb_cmd[COMMAND_ROLL]+stabilization_att_ff_cmd[COMMAND_ROLL]);
  stabilization_cmd[COMMAND_PITCH] =
    (int32_t)(stabilization_att_fb_cmd[COMMAND_PITCH]+stabilization_att_ff_cmd[COMMAND_PITCH]);
  stabilization_cmd[COMMAND_YAW] =
    (int32_t)(stabilization_att_fb_cmd[COMMAND_YAW]+stabilization_att_ff_cmd[COMMAND_YAW]);
    
  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);

}
