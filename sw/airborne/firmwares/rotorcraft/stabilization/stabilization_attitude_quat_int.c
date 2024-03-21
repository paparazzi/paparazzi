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

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"

/** explicitly define to zero to disable feed-forward rate term by default */
#ifndef STABILIZATION_ATTITUDE_PHI_FFDGAIN
#define STABILIZATION_ATTITUDE_PHI_FFDGAIN 0
#endif
#ifndef STABILIZATION_ATTITUDE_THETA_FFDGAIN
#define STABILIZATION_ATTITUDE_THETA_FFDGAIN 0
#endif
#ifndef STABILIZATION_ATTITUDE_PSI_FFDGAIN
#define STABILIZATION_ATTITUDE_PSI_FFDGAIN 0
#endif

struct Int32AttitudeGains stabilization_gains = {
  {STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
  {STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
  {STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
  {STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN },
  {STABILIZATION_ATTITUDE_PHI_FFDGAIN, STABILIZATION_ATTITUDE_THETA_FFDGAIN, STABILIZATION_ATTITUDE_PSI_FFDGAIN }
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
#error "ALL control gains have to be positive!!!"
#endif

struct Int32Quat stabilization_att_sum_err_quat;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

static struct Int32Quat   stab_att_sp_quat;
static struct Int32Eulers stab_att_sp_euler;

struct AttRefQuatInt att_ref_quat_i;

#define IERROR_SCALE 128
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 12
#define GAIN_PRESCALER_D 3
#define GAIN_PRESCALER_I 3

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)   //FIXME really use this message here ?
{
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  struct Int32Eulers *att = stateGetNedToBodyEulers_i();
  pprz_msg_send_STAB_ATTITUDE_INT(trans, dev, AC_ID,
                                  &(body_rate->p), &(body_rate->q), &(body_rate->r),
                                  &(att->phi), &(att->theta), &(att->psi),
                                  &stab_att_sp_euler.phi,
                                  &stab_att_sp_euler.theta,
                                  &stab_att_sp_euler.psi,
                                  &stabilization_att_sum_err_quat.qx,
                                  &stabilization_att_sum_err_quat.qy,
                                  &stabilization_att_sum_err_quat.qz,
                                  &stabilization_att_fb_cmd[COMMAND_ROLL],
                                  &stabilization_att_fb_cmd[COMMAND_PITCH],
                                  &stabilization_att_fb_cmd[COMMAND_YAW],
                                  &stabilization_att_ff_cmd[COMMAND_ROLL],
                                  &stabilization_att_ff_cmd[COMMAND_PITCH],
                                  &stabilization_att_ff_cmd[COMMAND_YAW],
                                  &stabilization.cmd[COMMAND_ROLL],
                                  &stabilization.cmd[COMMAND_PITCH],
                                  &stabilization.cmd[COMMAND_YAW]);
}

static void send_att_ref(struct transport_tx *trans, struct link_device *dev)
{
  // ref eulers in message are with REF_ANGLE_FRAC, convert
  struct Int32Eulers ref_euler;
  INT32_EULERS_LSHIFT(ref_euler, att_ref_quat_i.euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
  pprz_msg_send_STAB_ATTITUDE_REF_INT(trans, dev, AC_ID,
                                      &stab_att_sp_euler.phi,
                                      &stab_att_sp_euler.theta,
                                      &stab_att_sp_euler.psi,
                                      &ref_euler.phi,
                                      &ref_euler.theta,
                                      &ref_euler.psi,
                                      &att_ref_quat_i.rate.p,
                                      &att_ref_quat_i.rate.q,
                                      &att_ref_quat_i.rate.r,
                                      &att_ref_quat_i.accel.p,
                                      &att_ref_quat_i.accel.q,
                                      &att_ref_quat_i.accel.r);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &att_ref_quat_i.quat.qi,
                              &att_ref_quat_i.quat.qx,
                              &att_ref_quat_i.quat.qy,
                              &att_ref_quat_i.quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}
#endif

void stabilization_attitude_quat_int_init(void)
{
  attitude_ref_quat_int_init(&att_ref_quat_i);
  int32_quat_identity(&stabilization_att_sum_err_quat);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_INT, send_att);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_REF_INT, send_att_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
#endif
}

void stabilization_attitude_enter(void)
{
  struct Int32Quat *state_quat = stateGetNedToBodyQuat_i();
  attitude_ref_quat_int_enter(&att_ref_quat_i, state_quat);

  int32_quat_identity(&stabilization_att_sum_err_quat);
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
    GAIN_PRESCALER_P * gains->p.x  * QUAT1_FLOAT_OF_BFP(att_err->qx) +
    GAIN_PRESCALER_D * gains->d.x  * RATE_FLOAT_OF_BFP(rate_err->p) +
    GAIN_PRESCALER_I * gains->i.x  * QUAT1_FLOAT_OF_BFP(sum_err->qx);

  fb_commands[COMMAND_PITCH] =
    GAIN_PRESCALER_P * gains->p.y  * QUAT1_FLOAT_OF_BFP(att_err->qy) +
    GAIN_PRESCALER_D * gains->d.y  * RATE_FLOAT_OF_BFP(rate_err->q) +
    GAIN_PRESCALER_I * gains->i.y  * QUAT1_FLOAT_OF_BFP(sum_err->qy);

  fb_commands[COMMAND_YAW] =
    GAIN_PRESCALER_P * gains->p.z  * QUAT1_FLOAT_OF_BFP(att_err->qz) +
    GAIN_PRESCALER_D * gains->d.z  * RATE_FLOAT_OF_BFP(rate_err->r) +
    GAIN_PRESCALER_I * gains->i.z  * QUAT1_FLOAT_OF_BFP(sum_err->qz);

}

void stabilization_attitude_run(bool enable_integrator, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stab_att_sp_euler = stab_sp_to_eulers_i(sp);
  stab_att_sp_quat = stab_sp_to_quat_i(sp);

  /*
   * Update reference
   * Warning: dt is currently not used in the quat_int ref impl
   * PERIODIC_FREQUENCY is assumed to be 512Hz
   */
  static const float dt = (1./PERIODIC_FREQUENCY);
  attitude_ref_quat_int_update(&att_ref_quat_i, &stab_att_sp_quat, dt);

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
  int32_quat_inv_comp(&att_err, att_quat, &att_ref_quat_i.quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /*  rate error                */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(att_ref_quat_i.rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(att_ref_quat_i.rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(att_ref_quat_i.rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC))
  };
  struct Int32Rates rate_err;
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_scaled, (*body_rate));

#define INTEGRATOR_BOUND 100000
  /* integrated error */
  if (enable_integrator) {
    stabilization_att_sum_err_quat.qx += att_err.qx / IERROR_SCALE;
    stabilization_att_sum_err_quat.qy += att_err.qy / IERROR_SCALE;
    stabilization_att_sum_err_quat.qz += att_err.qz / IERROR_SCALE;
    Bound(stabilization_att_sum_err_quat.qx, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qy, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qz, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
  } else {
    /* reset accumulator */
    int32_quat_identity(&stabilization_att_sum_err_quat);
  }

  /* compute the feed forward command */
  attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains, &att_ref_quat_i.accel);

  /* compute the feed back command */
  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains, &att_err, &rate_err, &stabilization_att_sum_err_quat);

  /* sum feedforward and feedback */
  cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL] + stabilization_att_ff_cmd[COMMAND_ROLL];
  cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH] + stabilization_att_ff_cmd[COMMAND_PITCH];
  cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW] + stabilization_att_ff_cmd[COMMAND_YAW];
  cmd[COMMAND_THRUST] = th_sp_to_thrust_i(thrust, 0, THRUST_AXIS_Z);

  /* bound the result */
  BoundAbs(cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_YAW], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_THRUST], MAX_PPRZ);
}

