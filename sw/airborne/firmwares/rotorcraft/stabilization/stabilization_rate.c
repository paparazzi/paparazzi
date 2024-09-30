/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2010 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_rate.c
 *  Rate stabilization for rotorcrafts.
 *
 *  Control loops for angular velocity.
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"

#include "state.h"

#include "modules/imu/imu.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

#define MAX_SUM_ERR 40000

#ifndef STABILIZATION_RATE_IGAIN_P
#define STABILIZATION_RATE_IGAIN_P 0
#endif

#ifndef STABILIZATION_RATE_IGAIN_Q
#define STABILIZATION_RATE_IGAIN_Q 0
#endif

#ifndef STABILIZATION_RATE_IGAIN_R
#define STABILIZATION_RATE_IGAIN_R 0
#endif

#if (STABILIZATION_RATE_GAIN_P < 0) || \
  (STABILIZATION_RATE_GAIN_Q < 0)   || \
  (STABILIZATION_RATE_GAIN_R < 0)   || \
  (STABILIZATION_RATE_IGAIN_P < 0)  || \
  (STABILIZATION_RATE_IGAIN_Q < 0)  || \
  (STABILIZATION_RATE_IGAIN_R < 0)
#error "ALL control gains have to be positive!!!"
#endif

// divide by 2^_b and round instead of floor
#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

static struct FloatRates stabilization_rate_sp;
static struct FloatRates stabilization_rate_sum_err;
static struct FloatRates stabilization_rate_fb_cmd;
struct FloatRates stabilization_rate_gain;
struct FloatRates stabilization_rate_igain;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_rate(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_RATE_LOOP(trans, dev, AC_ID,
                          &stabilization_rate_sp.p,
                          &stabilization_rate_sp.q,
                          &stabilization_rate_sp.r,
                          &stabilization_rate_sum_err.p,
                          &stabilization_rate_sum_err.q,
                          &stabilization_rate_sum_err.r,
                          &stabilization_rate_fb_cmd.p,
                          &stabilization_rate_fb_cmd.q,
                          &stabilization_rate_fb_cmd.r,
                          &stabilization.cmd[COMMAND_THRUST]);
}
#endif

void stabilization_rate_init(void)
{

  FLOAT_RATES_ZERO(stabilization_rate_sp);

  RATES_ASSIGN(stabilization_rate_gain,
               STABILIZATION_RATE_GAIN_P,
               STABILIZATION_RATE_GAIN_Q,
               STABILIZATION_RATE_GAIN_R);
  RATES_ASSIGN(stabilization_rate_igain,
               STABILIZATION_RATE_IGAIN_P,
               STABILIZATION_RATE_IGAIN_Q,
               STABILIZATION_RATE_IGAIN_R);;

  FLOAT_RATES_ZERO(stabilization_rate_sum_err);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RATE_LOOP, send_rate);
#endif
}

void stabilization_rate_enter(void)
{
  FLOAT_RATES_ZERO(stabilization_rate_sum_err);
}

void stabilization_rate_run(bool in_flight, struct StabilizationSetpoint *rate_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  /* compute feed-back command */
  struct FloatRates _error;
  struct FloatRates *body_rate = stateGetBodyRates_f();
  stabilization_rate_sp = stab_sp_to_rates_f(rate_sp);
  RATES_DIFF(_error, stabilization_rate_sp, (*body_rate));
  if (in_flight) {
    /* update integrator */
    //divide the sum_err_increment to make sure it doesn't accumulate to the max too fast
    struct FloatRates sum_err_increment;
    RATES_SDIV(sum_err_increment, _error, PERIODIC_FREQUENCY);
    RATES_ADD(stabilization_rate_sum_err, sum_err_increment);
    RATES_BOUND_CUBE(stabilization_rate_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  } else {
    FLOAT_RATES_ZERO(stabilization_rate_sum_err);
  }

  /* PI */
  stabilization_rate_fb_cmd.p = stabilization_rate_gain.p * _error.p +
                                stabilization_rate_igain.p  * stabilization_rate_sum_err.p;

  stabilization_rate_fb_cmd.q = stabilization_rate_gain.q * _error.q +
                                stabilization_rate_igain.q  * stabilization_rate_sum_err.q;

  stabilization_rate_fb_cmd.r = stabilization_rate_gain.r * _error.r +
                                stabilization_rate_igain.r  * stabilization_rate_sum_err.r;

  cmd[COMMAND_ROLL]  = stabilization_rate_fb_cmd.p;
  cmd[COMMAND_PITCH] = stabilization_rate_fb_cmd.q;
  cmd[COMMAND_YAW]   = stabilization_rate_fb_cmd.r;
  cmd[COMMAND_THRUST] = th_sp_to_thrust_i(thrust, 0, THRUST_AXIS_Z);

  /* bound the result */
  BoundAbs(cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_YAW], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_THRUST], MAX_PPRZ);
}

struct StabilizationSetpoint stabilization_rate_read_rc(struct RadioControl *rc)
{
  struct FloatRates rate_sp;
  FLOAT_RATES_ZERO(rate_sp);
  if (ROLL_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.p = rc->values[RC_RATE_P] * STABILIZATION_RATE_SP_MAX_P / MAX_PPRZ;
  }
  if (PITCH_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.q = rc->values[RC_RATE_Q] * STABILIZATION_RATE_SP_MAX_Q / MAX_PPRZ;
  }
  if (YAW_RATE_DEADBAND_EXCEEDED(rc) && !THROTTLE_STICK_DOWN_FROM_RC(rc)) {
    rate_sp.r = rc->values[RC_RATE_R] * STABILIZATION_RATE_SP_MAX_R / MAX_PPRZ;
  }
  return stab_sp_from_rates_f(&rate_sp);
}

