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

struct Int32AttitudeGains  stabilization_gains;

/* warn if some gains are still negative */
#if (STABILIZATION_ATTITUDE_PHI_PGAIN < 0) || \
  (STABILIZATION_ATTITUDE_THETA_PGAIN < 0) || \
  (STABILIZATION_ATTITUDE_PSI_PGAIN < 0)   || \
  (STABILIZATION_ATTITUDE_PHI_DGAIN < 0)   || \
  (STABILIZATION_ATTITUDE_THETA_DGAIN < 0) || \
  (STABILIZATION_ATTITUDE_PSI_DGAIN < 0)   || \
  (STABILIZATION_ATTITUDE_PHI_IGAIN < 0)   || \
  (STABILIZATION_ATTITUDE_THETA_IGAIN < 0) || \
  (STABILIZATION_ATTITUDE_PSI_IGAIN  < 0)
#warning "ALL control gains are now positive!!!"
#endif

struct Int32Eulers stabilization_att_sum_err;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();


  VECT3_ASSIGN(stabilization_gains.p,
               STABILIZATION_ATTITUDE_PHI_PGAIN,
               STABILIZATION_ATTITUDE_THETA_PGAIN,
               STABILIZATION_ATTITUDE_PSI_PGAIN);

  VECT3_ASSIGN(stabilization_gains.d,
               STABILIZATION_ATTITUDE_PHI_DGAIN,
               STABILIZATION_ATTITUDE_THETA_DGAIN,
               STABILIZATION_ATTITUDE_PSI_DGAIN);

  VECT3_ASSIGN(stabilization_gains.i,
               STABILIZATION_ATTITUDE_PHI_IGAIN,
               STABILIZATION_ATTITUDE_THETA_IGAIN,
               STABILIZATION_ATTITUDE_PSI_IGAIN);

  VECT3_ASSIGN(stabilization_gains.dd,
               STABILIZATION_ATTITUDE_PHI_DDGAIN,
               STABILIZATION_ATTITUDE_THETA_DDGAIN,
               STABILIZATION_ATTITUDE_PSI_DDGAIN);


  INT_EULERS_ZERO( stabilization_att_sum_err );

}


void stabilization_attitude_read_rc(bool_t in_flight) {

  stabilization_attitude_read_rc_setpoint_eulers(&stab_att_sp_euler, in_flight);

}


void stabilization_attitude_enter(void) {

  stab_att_sp_euler.psi = stateGetNedToBodyEulers_i()->psi;
  reset_psi_ref_from_body();
  INT_EULERS_ZERO( stabilization_att_sum_err );

}


#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

#define MAX_SUM_ERR 4000000

void stabilization_attitude_run(bool_t  in_flight) {


  /* update reference */
  stabilization_attitude_ref_update();

  /* compute feedforward command */
  stabilization_att_ff_cmd[COMMAND_ROLL] =
    OFFSET_AND_ROUND(stabilization_gains.dd.x * stab_att_ref_accel.p, 5);
  stabilization_att_ff_cmd[COMMAND_PITCH] =
    OFFSET_AND_ROUND(stabilization_gains.dd.y * stab_att_ref_accel.q, 5);
  stabilization_att_ff_cmd[COMMAND_YAW] =
    OFFSET_AND_ROUND(stabilization_gains.dd.z * stab_att_ref_accel.r, 5);

  /* compute feedback command */
  /* attitude error            */
  const struct Int32Eulers att_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_euler.phi,   (REF_ANGLE_FRAC - INT32_ANGLE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_euler.theta, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_euler.psi,   (REF_ANGLE_FRAC - INT32_ANGLE_FRAC)) };
  struct Int32Eulers att_err;
  struct Int32Eulers* ltp_to_body_euler = stateGetNedToBodyEulers_i();
  EULERS_DIFF(att_err, att_ref_scaled, (*ltp_to_body_euler));
  INT32_ANGLE_NORMALIZE(att_err.psi);

  if (in_flight) {
    /* update integrator */
    EULERS_ADD(stabilization_att_sum_err, att_err);
    EULERS_BOUND_CUBE(stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  }
  else {
    INT_EULERS_ZERO(stabilization_att_sum_err);
  }

  /* rate error                */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC)) };
  struct Int32Rates rate_err;
  struct Int32Rates* body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_scaled, (*body_rate));

  /* PID                  */
  stabilization_att_fb_cmd[COMMAND_ROLL] =
    stabilization_gains.p.x    * att_err.phi +
    stabilization_gains.d.x    * rate_err.p +
    OFFSET_AND_ROUND2((stabilization_gains.i.x  * stabilization_att_sum_err.phi), 10);

  stabilization_att_fb_cmd[COMMAND_PITCH] =
    stabilization_gains.p.y    * att_err.theta +
    stabilization_gains.d.y    * rate_err.q +
    OFFSET_AND_ROUND2((stabilization_gains.i.y  * stabilization_att_sum_err.theta), 10);

  stabilization_att_fb_cmd[COMMAND_YAW] =
    stabilization_gains.p.z    * att_err.psi +
    stabilization_gains.d.z    * rate_err.r +
    OFFSET_AND_ROUND2((stabilization_gains.i.z  * stabilization_att_sum_err.psi), 10);


  /* with P gain of 100, att_err of 180deg (3.14 rad)
   * fb cmd: 100 * 3.14 * 2^12 / 2^CMD_SHIFT = 628
   * max possible command is 9600
   */
#define CMD_SHIFT 11

  /* sum feedforward and feedback */
  stabilization_cmd[COMMAND_ROLL] =
    OFFSET_AND_ROUND((stabilization_att_fb_cmd[COMMAND_ROLL]+stabilization_att_ff_cmd[COMMAND_ROLL]), CMD_SHIFT);

  stabilization_cmd[COMMAND_PITCH] =
    OFFSET_AND_ROUND((stabilization_att_fb_cmd[COMMAND_PITCH]+stabilization_att_ff_cmd[COMMAND_PITCH]), CMD_SHIFT);

  stabilization_cmd[COMMAND_YAW] =
    OFFSET_AND_ROUND((stabilization_att_fb_cmd[COMMAND_YAW]+stabilization_att_ff_cmd[COMMAND_YAW]), CMD_SHIFT);

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);

}
