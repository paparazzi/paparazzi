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

/**
 * @file stabilization_attitude_euler_float.c
 *
 * Rotorcraft attitude stabilization in euler float version.
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_euler_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.h"

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

#ifndef USE_ATT_REF
#define USE_ATT_REF 1
#endif

struct FloatAttitudeGains stabilization_gains;
struct FloatEulers stabilization_att_sum_err;

static struct FloatEulers stab_att_sp_euler;
static struct AttRefEulerFloat att_ref_euler_f;

float stabilization_att_fb_cmd[COMMANDS_NB];
float stabilization_att_ff_cmd[COMMANDS_NB];

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_att(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatRates *body_rate = stateGetBodyRates_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float foo = 0.0;
  pprz_msg_send_STAB_ATTITUDE_FLOAT(trans, dev, AC_ID,
                                    &(body_rate->p), &(body_rate->q), &(body_rate->r),
                                    &(att->phi), &(att->theta), &(att->psi),
                                    &stab_att_sp_euler.phi,
                                    &stab_att_sp_euler.theta,
                                    &stab_att_sp_euler.psi,
                                    &stabilization_att_sum_err.phi,
                                    &stabilization_att_sum_err.theta,
                                    &stabilization_att_sum_err.psi,
                                    &stabilization_att_fb_cmd[COMMAND_ROLL],
                                    &stabilization_att_fb_cmd[COMMAND_PITCH],
                                    &stabilization_att_fb_cmd[COMMAND_YAW],
                                    &stabilization_att_ff_cmd[COMMAND_ROLL],
                                    &stabilization_att_ff_cmd[COMMAND_PITCH],
                                    &stabilization_att_ff_cmd[COMMAND_YAW],
                                    &stabilization.cmd[COMMAND_ROLL],
                                    &stabilization.cmd[COMMAND_PITCH],
                                    &stabilization.cmd[COMMAND_YAW],
                                    &foo, &foo, &foo);
}

static void send_att_ref(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_REF_FLOAT(trans, dev, AC_ID,
                                        &stab_att_sp_euler.phi,
                                        &stab_att_sp_euler.theta,
                                        &stab_att_sp_euler.psi,
                                        &att_ref_euler_f.euler.phi,
                                        &att_ref_euler_f.euler.theta,
                                        &att_ref_euler_f.euler.psi,
                                        &att_ref_euler_f.rate.p,
                                        &att_ref_euler_f.rate.q,
                                        &att_ref_euler_f.rate.r,
                                        &att_ref_euler_f.accel.p,
                                        &att_ref_euler_f.accel.q,
                                        &att_ref_euler_f.accel.r);
}
#endif

void stabilization_attitude_euler_float_init(void)
{

  attitude_ref_euler_float_init(&att_ref_euler_f);

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

  FLOAT_EULERS_ZERO(stabilization_att_sum_err);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_FLOAT, send_att);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_REF_FLOAT, send_att_ref);
#endif
}

void stabilization_attitude_enter(void)
{
  /* reset psi ref to current psi angle */
  attitude_ref_euler_float_enter(&att_ref_euler_f, stabilization_attitude_get_heading_f());

  FLOAT_EULERS_ZERO(stabilization_att_sum_err);
}

#define MAX_SUM_ERR 200

void stabilization_attitude_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stab_att_sp_euler = stab_sp_to_eulers_f(sp);
#if USE_ATT_REF
  static const float dt = (1./PERIODIC_FREQUENCY);
  attitude_ref_euler_float_update(&att_ref_euler_f, &stab_att_sp_euler, dt);
#else
  EULERS_COPY(att_ref_euler_f.euler, stab_att_sp_euler);
  FLOAT_RATES_ZERO(att_ref_euler_f.rate);
  FLOAT_RATES_ZERO(att_ref_euler_f.accel);
#endif

  /* Compute feedforward */
  stabilization_att_ff_cmd[COMMAND_ROLL] =
    stabilization_gains.dd.x * att_ref_euler_f.accel.p;
  stabilization_att_ff_cmd[COMMAND_PITCH] =
    stabilization_gains.dd.y * att_ref_euler_f.accel.q;
  stabilization_att_ff_cmd[COMMAND_YAW] =
    stabilization_gains.dd.z * att_ref_euler_f.accel.r;

  /* Compute feedback                  */
  /* attitude error            */
  struct FloatEulers *att_float = stateGetNedToBodyEulers_f();
  struct FloatEulers att_err;
  EULERS_DIFF(att_err, att_ref_euler_f.euler, *att_float);
  FLOAT_ANGLE_NORMALIZE(att_err.psi);

  if (in_flight) {
    /* update integrator */
    EULERS_ADD(stabilization_att_sum_err, att_err);
    EULERS_BOUND_CUBE(stabilization_att_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  } else {
    FLOAT_EULERS_ZERO(stabilization_att_sum_err);
  }

  /*  rate error                */
  struct FloatRates *rate_float = stateGetBodyRates_f();
  struct FloatRates rate_err;
  RATES_DIFF(rate_err, att_ref_euler_f.rate, *rate_float);

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
