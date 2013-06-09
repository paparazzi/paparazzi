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

/** @file stabilization_attitude_quat_int.c
 * Rotorcraft quaternion attitude stabilization
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"

struct Int32AttitudeGains stabilization_gains = {
  {STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
  {STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
  {STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
  {STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
};

/* warn if some gains are still negative */
#if (STABILIZATION_ATTITUDE_PHI_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_THETA_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_PGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_DGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_IGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_IGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_IGAIN  < 0)
#warning "ALL control gains are now positive!!!"
#endif

struct Int32Quat stabilization_att_sum_err_quat;
struct Int32Eulers stabilization_att_sum_err;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

#define IERROR_SCALE 1024
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 48
#define GAIN_PRESCALER_D 48
#define GAIN_PRESCALER_I 48

void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();

  INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
  INT_EULERS_ZERO( stabilization_att_sum_err );
}

void stabilization_attitude_enter(void) {

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  INT32_QUAT_ZERO(stabilization_att_sum_err_quat);
  INT_EULERS_ZERO(stabilization_att_sum_err);
}

void stabilization_attitude_set_failsafe_setpoint(void) {
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_from_eulers_i(struct Int32Eulers *sp_euler) {
  // copy euler setpoint for debugging
  memcpy(&stab_att_sp_euler, sp_euler, sizeof(struct Int32Eulers));
  INT32_QUAT_OF_EULERS(stab_att_sp_quat, *sp_euler);
  INT32_QUAT_WRAP_SHORTEST(stab_att_sp_quat);
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

static void attitude_run_ff(int32_t ff_commands[], struct Int32AttitudeGains *gains, struct Int32Rates *ref_accel)
{
  /* Compute feedforward based on reference acceleration */

  ff_commands[COMMAND_ROLL]  = GAIN_PRESCALER_FF * gains->dd.x * RATE_FLOAT_OF_BFP(ref_accel->p) / (1 << 7);
  ff_commands[COMMAND_PITCH] = GAIN_PRESCALER_FF * gains->dd.y * RATE_FLOAT_OF_BFP(ref_accel->q) / (1 << 7);
  ff_commands[COMMAND_YAW]   = GAIN_PRESCALER_FF * gains->dd.z * RATE_FLOAT_OF_BFP(ref_accel->r) / (1 << 7);
}

static void attitude_run_fb(int32_t fb_commands[], struct Int32AttitudeGains *gains, struct Int32Quat *att_err,
    struct Int32Rates *rate_err, struct Int32Quat *sum_err)
{
  /*  PID feedback */
  fb_commands[COMMAND_ROLL] =
    GAIN_PRESCALER_P * gains->p.x  * QUAT1_FLOAT_OF_BFP(att_err->qx) / 4 +
    GAIN_PRESCALER_D * gains->d.x  * RATE_FLOAT_OF_BFP(rate_err->p) / 16 +
    GAIN_PRESCALER_I * gains->i.x  * QUAT1_FLOAT_OF_BFP(sum_err->qx) / 2;

  fb_commands[COMMAND_PITCH] =
    GAIN_PRESCALER_P * gains->p.y  * QUAT1_FLOAT_OF_BFP(att_err->qy) / 4 +
    GAIN_PRESCALER_D * gains->d.y  * RATE_FLOAT_OF_BFP(rate_err->q)  / 16 +
    GAIN_PRESCALER_I * gains->i.y  * QUAT1_FLOAT_OF_BFP(sum_err->qy) / 2;

  fb_commands[COMMAND_YAW] =
    GAIN_PRESCALER_P * gains->p.z  * QUAT1_FLOAT_OF_BFP(att_err->qz) / 4 +
    GAIN_PRESCALER_D * gains->d.z  * RATE_FLOAT_OF_BFP(rate_err->r)  / 16 +
    GAIN_PRESCALER_I * gains->i.z  * QUAT1_FLOAT_OF_BFP(sum_err->qz) / 2;

}

void stabilization_attitude_run(bool_t enable_integrator) {

  /*
   * Update reference
   */
  stabilization_attitude_ref_update();

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat* att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  INT32_QUAT_WRAP_SHORTEST(att_err);
  INT32_QUAT_NORMALIZE(att_err);

  /*  rate error                */
  struct Int32Rates rate_err;
  struct Int32Rates* body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, stab_att_ref_rate, *body_rate);

  /* integrated error */
  if (enable_integrator) {
    struct Int32Quat new_sum_err, scaled_att_err;
    /* update accumulator */
    scaled_att_err.qi = att_err.qi;
    scaled_att_err.qx = att_err.qx / IERROR_SCALE;
    scaled_att_err.qy = att_err.qy / IERROR_SCALE;
    scaled_att_err.qz = att_err.qz / IERROR_SCALE;
    INT32_QUAT_COMP(new_sum_err, stabilization_att_sum_err_quat, scaled_att_err);
    INT32_QUAT_NORMALIZE(new_sum_err);
    QUAT_COPY(stabilization_att_sum_err_quat, new_sum_err);
    INT32_EULERS_OF_QUAT(stabilization_att_sum_err, stabilization_att_sum_err_quat);
  } else {
    /* reset accumulator */
    INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
    INT_EULERS_ZERO( stabilization_att_sum_err );
  }

  /* compute the feed forward command */
  attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains, &stab_att_ref_accel);

  /* compute the feed back command */
  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains, &att_err, &rate_err, &stabilization_att_sum_err_quat);

  /* sum feedforward and feedback */
  stabilization_cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL] + stabilization_att_ff_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH] + stabilization_att_ff_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW] + stabilization_att_ff_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_read_rc(bool_t in_flight) {
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}
