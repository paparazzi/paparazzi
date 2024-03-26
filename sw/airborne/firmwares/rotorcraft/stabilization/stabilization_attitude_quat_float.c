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

/** @file firmwares/rotorcraft/stabilization/stabilization_attitude_quat_float.c
 * @brief Quaternion attitude stabilization (floating point).
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"

struct FloatAttitudeGains stabilization_gains[STABILIZATION_ATTITUDE_GAIN_NB];

static struct FloatEulers stab_att_sp_euler;
static struct FloatQuat   stab_att_sp_quat;

struct AttRefQuatFloat att_ref_quat_f;

struct FloatQuat stabilization_att_sum_err_quat;

static struct FloatRates last_body_rate;
static struct FloatRates body_rate_d;

float stabilization_att_fb_cmd[COMMANDS_NB];
float stabilization_att_ff_cmd[COMMANDS_NB];

static int gain_idx = STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT;

static const float phi_pgain[] = STABILIZATION_ATTITUDE_PHI_PGAIN;
static const float theta_pgain[] = STABILIZATION_ATTITUDE_THETA_PGAIN;
static const float psi_pgain[] = STABILIZATION_ATTITUDE_PSI_PGAIN;

static const float phi_dgain[] = STABILIZATION_ATTITUDE_PHI_DGAIN;
static const float theta_dgain[] = STABILIZATION_ATTITUDE_THETA_DGAIN;
static const float psi_dgain[] = STABILIZATION_ATTITUDE_PSI_DGAIN;

static const float phi_igain[] = STABILIZATION_ATTITUDE_PHI_IGAIN;
static const float theta_igain[] = STABILIZATION_ATTITUDE_THETA_IGAIN;
static const float psi_igain[] = STABILIZATION_ATTITUDE_PSI_IGAIN;

static const float phi_ddgain[] = STABILIZATION_ATTITUDE_PHI_DDGAIN;
static const float theta_ddgain[] = STABILIZATION_ATTITUDE_THETA_DDGAIN;
static const float psi_ddgain[] = STABILIZATION_ATTITUDE_PSI_DDGAIN;

static const float phi_dgain_d[] = STABILIZATION_ATTITUDE_PHI_DGAIN_D;
static const float theta_dgain_d[] = STABILIZATION_ATTITUDE_THETA_DGAIN_D;
static const float psi_dgain_d[] = STABILIZATION_ATTITUDE_PSI_DGAIN_D;


#if defined COMMAND_ROLL_SURFACE && defined COMMAND_PITCH_SURFACE && defined COMMAND_YAW_SURFACE
#define HAS_SURFACE_COMMANDS 1
#endif

#ifdef HAS_SURFACE_COMMANDS
static const float phi_pgain_surface[] = STABILIZATION_ATTITUDE_PHI_PGAIN_SURFACE;
static const float theta_pgain_surface[] = STABILIZATION_ATTITUDE_THETA_PGAIN_SURFACE;
static const float psi_pgain_surface[] = STABILIZATION_ATTITUDE_PSI_PGAIN_SURFACE;

static const float phi_dgain_surface[] = STABILIZATION_ATTITUDE_PHI_DGAIN_SURFACE;
static const float theta_dgain_surface[] = STABILIZATION_ATTITUDE_THETA_DGAIN_SURFACE;
static const float psi_dgain_surface[] = STABILIZATION_ATTITUDE_PSI_DGAIN_SURFACE;

static const float phi_igain_surface[] = STABILIZATION_ATTITUDE_PHI_IGAIN_SURFACE;
static const float theta_igain_surface[] = STABILIZATION_ATTITUDE_THETA_IGAIN_SURFACE;
static const float psi_igain_surface[] = STABILIZATION_ATTITUDE_PSI_IGAIN_SURFACE;

static const float phi_ddgain_surface[] = STABILIZATION_ATTITUDE_PHI_DDGAIN_SURFACE;
static const float theta_ddgain_surface[] = STABILIZATION_ATTITUDE_THETA_DDGAIN_SURFACE;
static const float psi_ddgain_surface[] = STABILIZATION_ATTITUDE_PSI_DDGAIN_SURFACE;
#endif

#define IERROR_SCALE 1024

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatRates *body_rate = stateGetBodyRates_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  pprz_msg_send_STAB_ATTITUDE_FLOAT(trans, dev, AC_ID,
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
                                    &stabilization.cmd[COMMAND_YAW],
                                    &body_rate_d.p, &body_rate_d.q, &body_rate_d.r);
}

static void send_att_ref(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_REF_FLOAT(trans, dev, AC_ID,
                                        &stab_att_sp_euler.phi,
                                        &stab_att_sp_euler.theta,
                                        &stab_att_sp_euler.psi,
                                        &att_ref_quat_f.euler.phi,
                                        &att_ref_quat_f.euler.theta,
                                        &att_ref_quat_f.euler.psi,
                                        &att_ref_quat_f.rate.p,
                                        &att_ref_quat_f.rate.q,
                                        &att_ref_quat_f.rate.r,
                                        &att_ref_quat_f.accel.p,
                                        &att_ref_quat_f.accel.q,
                                        &att_ref_quat_f.accel.r);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  struct Int32Quat refquat;
  QUAT_BFP_OF_REAL(refquat, att_ref_quat_f.quat);
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &refquat.qi,
                              &refquat.qx,
                              &refquat.qy,
                              &refquat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}
#endif

void stabilization_attitude_quat_float_init(void)
{
  /* setpoints */
  FLOAT_EULERS_ZERO(stab_att_sp_euler);
  float_quat_identity(&stab_att_sp_quat);
  /* reference */
  attitude_ref_quat_float_init(&att_ref_quat_f);
  attitude_ref_quat_float_schedule(&att_ref_quat_f, STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT);

  for (int i = 0; i < STABILIZATION_ATTITUDE_GAIN_NB; i++) {
    VECT3_ASSIGN(stabilization_gains[i].p, phi_pgain[i], theta_pgain[i], psi_pgain[i]);
    VECT3_ASSIGN(stabilization_gains[i].d, phi_dgain[i], theta_dgain[i], psi_dgain[i]);
    VECT3_ASSIGN(stabilization_gains[i].i, phi_igain[i], theta_igain[i], psi_igain[i]);
    VECT3_ASSIGN(stabilization_gains[i].dd, phi_ddgain[i], theta_ddgain[i], psi_ddgain[i]);
    VECT3_ASSIGN(stabilization_gains[i].rates_d, phi_dgain_d[i], theta_dgain_d[i], psi_dgain_d[i]);
#ifdef HAS_SURFACE_COMMANDS
    VECT3_ASSIGN(stabilization_gains[i].surface_p, phi_pgain_surface[i], theta_pgain_surface[i], psi_pgain_surface[i]);
    VECT3_ASSIGN(stabilization_gains[i].surface_d, phi_dgain_surface[i], theta_dgain_surface[i], psi_dgain_surface[i]);
    VECT3_ASSIGN(stabilization_gains[i].surface_i, phi_igain_surface[i], theta_igain_surface[i], psi_igain_surface[i]);
    VECT3_ASSIGN(stabilization_gains[i].surface_dd, phi_ddgain_surface[i], theta_ddgain_surface[i], psi_ddgain_surface[i]);
#endif
  }

  float_quat_identity(&stabilization_att_sum_err_quat);
  FLOAT_RATES_ZERO(last_body_rate);
  FLOAT_RATES_ZERO(body_rate_d);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_FLOAT, send_att);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_REF_FLOAT, send_att_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
#endif
}

void stabilization_attitude_gain_schedule(uint8_t idx)
{
  if (gain_idx >= STABILIZATION_ATTITUDE_GAIN_NB) {
    // This could be bad -- Just say no.
    return;
  }
  gain_idx = idx;
  attitude_ref_quat_float_schedule(&att_ref_quat_f, idx);
}

void stabilization_attitude_enter(void)
{
  struct FloatQuat *state_quat = stateGetNedToBodyQuat_f();
  attitude_ref_quat_float_enter(&att_ref_quat_f, state_quat);

  float_quat_identity(&stabilization_att_sum_err_quat);
}

#ifndef GAIN_PRESCALER_FF
#define GAIN_PRESCALER_FF 1
#endif
static void attitude_run_ff(float ff_commands[], struct FloatAttitudeGains *gains, struct FloatRates *ref_accel)
{
  /* Compute feedforward based on reference acceleration */

  ff_commands[COMMAND_ROLL]          = GAIN_PRESCALER_FF * gains->dd.x * ref_accel->p;
  ff_commands[COMMAND_PITCH]         = GAIN_PRESCALER_FF * gains->dd.y * ref_accel->q;
  ff_commands[COMMAND_YAW]           = GAIN_PRESCALER_FF * gains->dd.z * ref_accel->r;
#ifdef HAS_SURFACE_COMMANDS
  ff_commands[COMMAND_ROLL_SURFACE]  = GAIN_PRESCALER_FF * gains->surface_dd.x * ref_accel->p;
  ff_commands[COMMAND_PITCH_SURFACE] = GAIN_PRESCALER_FF * gains->surface_dd.y * ref_accel->q;
  ff_commands[COMMAND_YAW_SURFACE]   = GAIN_PRESCALER_FF * gains->surface_dd.z * ref_accel->r;
#endif
}

#ifndef GAIN_PRESCALER_P
#define GAIN_PRESCALER_P 1
#endif
#ifndef GAIN_PRESCALER_D
#define GAIN_PRESCALER_D 1
#endif
#ifndef GAIN_PRESCALER_I
#define GAIN_PRESCALER_I 1
#endif
static void attitude_run_fb(float fb_commands[], struct FloatAttitudeGains *gains, struct FloatQuat *att_err,
                            struct FloatRates *rate_err, struct FloatRates *rate_err_d, struct FloatQuat *sum_err)
{
  /*  PID feedback */
  fb_commands[COMMAND_ROLL] =
    GAIN_PRESCALER_P * gains->p.x  * att_err->qx +
    GAIN_PRESCALER_D * gains->d.x  * rate_err->p +
    GAIN_PRESCALER_D * gains->rates_d.x  * rate_err_d->p +
    GAIN_PRESCALER_I * gains->i.x  * sum_err->qx;

  fb_commands[COMMAND_PITCH] =
    GAIN_PRESCALER_P * gains->p.y  * att_err->qy +
    GAIN_PRESCALER_D * gains->d.y  * rate_err->q +
    GAIN_PRESCALER_D * gains->rates_d.y  * rate_err_d->q +
    GAIN_PRESCALER_I * gains->i.y  * sum_err->qy;

  fb_commands[COMMAND_YAW] =
    GAIN_PRESCALER_P * gains->p.z  * att_err->qz +
    GAIN_PRESCALER_D * gains->d.z  * rate_err->r +
    GAIN_PRESCALER_D * gains->rates_d.z  * rate_err_d->r +
    GAIN_PRESCALER_I * gains->i.z  * sum_err->qz;

#ifdef HAS_SURFACE_COMMANDS
  fb_commands[COMMAND_ROLL_SURFACE] =
    GAIN_PRESCALER_P * gains->surface_p.x  * att_err->qx +
    GAIN_PRESCALER_D * gains->surface_d.x  * rate_err->p +
    GAIN_PRESCALER_I * gains->surface_i.x  * sum_err->qx;

  fb_commands[COMMAND_PITCH_SURFACE] =
    GAIN_PRESCALER_P * gains->surface_p.y  * att_err->qy +
    GAIN_PRESCALER_D * gains->surface_d.y  * rate_err->q +
    GAIN_PRESCALER_I * gains->surface_i.y  * sum_err->qy;

  fb_commands[COMMAND_YAW_SURFACE] =
    GAIN_PRESCALER_P * gains->surface_p.z  * att_err->qz +
    GAIN_PRESCALER_D * gains->surface_d.z  * rate_err->r +
    GAIN_PRESCALER_I * gains->surface_i.z  * sum_err->qz;
#endif
}

void stabilization_attitude_run(bool enable_integrator, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stab_att_sp_euler = stab_sp_to_eulers_f(sp);
  stab_att_sp_quat = stab_sp_to_quat_f(sp);

  /*
   * Update reference
   */
  static const float dt = (1./PERIODIC_FREQUENCY);
  attitude_ref_quat_float_update(&att_ref_quat_f, &stab_att_sp_quat, dt);

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct FloatQuat att_err;
  struct FloatQuat *att_quat = stateGetNedToBodyQuat_f();
  float_quat_inv_comp(&att_err, att_quat, &att_ref_quat_f.quat);
  /* wrap it in the shortest direction       */
  float_quat_wrap_shortest(&att_err);

  /*  rate error                */
  struct FloatRates rate_err;
  struct FloatRates *body_rate = stateGetBodyRates_f();
  RATES_DIFF(rate_err, att_ref_quat_f.rate, *body_rate);
  /* rate_d error               */
  RATES_DIFF(body_rate_d, *body_rate, last_body_rate);
  RATES_COPY(last_body_rate, *body_rate);

#define INTEGRATOR_BOUND 1.0
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
    float_quat_identity(&stabilization_att_sum_err_quat);
  }

  attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains[gain_idx], &att_ref_quat_f.accel);

  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains[gain_idx], &att_err, &rate_err, &body_rate_d,
                  &stabilization_att_sum_err_quat);

  cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL] + stabilization_att_ff_cmd[COMMAND_ROLL];
  cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH] + stabilization_att_ff_cmd[COMMAND_PITCH];
  cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW] + stabilization_att_ff_cmd[COMMAND_YAW];
  cmd[COMMAND_THRUST] = th_sp_to_thrust_i(thrust, 0, THRUST_AXIS_Z);

#ifdef HAS_SURFACE_COMMANDS
  cmd[COMMAND_ROLL_SURFACE] = stabilization_att_fb_cmd[COMMAND_ROLL_SURFACE] +
      stabilization_att_ff_cmd[COMMAND_ROLL_SURFACE];
  cmd[COMMAND_PITCH_SURFACE] = stabilization_att_fb_cmd[COMMAND_PITCH_SURFACE] +
      stabilization_att_ff_cmd[COMMAND_PITCH_SURFACE];
  cmd[COMMAND_YAW_SURFACE] = stabilization_att_fb_cmd[COMMAND_YAW_SURFACE] +
      stabilization_att_ff_cmd[COMMAND_YAW_SURFACE];
#endif

  /* bound the result */
  BoundAbs(cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_YAW], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_THRUST], MAX_PPRZ);
#ifdef HAS_SURFACE_COMMANDS
  BoundAbs(cmd[COMMAND_ROLL_SURFACE], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_PITCH_SURFACE], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_YAW_SURFACE], MAX_PPRZ);
#endif
}

