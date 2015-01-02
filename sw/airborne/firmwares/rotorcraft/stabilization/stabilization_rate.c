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

#include "subsystems/imu.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

#define F_UPDATE_RES 9
#define REF_DOT_FRAC 11
#define REF_FRAC  16

#define MAX_SUM_ERR 4000000

#ifndef STABILIZATION_RATE_DDGAIN_P
#define STABILIZATION_RATE_DDGAIN_P 0
#endif
#ifndef STABILIZATION_RATE_DDGAIN_Q
#define STABILIZATION_RATE_DDGAIN_Q 0
#endif
#ifndef STABILIZATION_RATE_DDGAIN_R
#define STABILIZATION_RATE_DDGAIN_R 0
#endif

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

#ifndef STABILIZATION_RATE_REF_TAU
#define STABILIZATION_RATE_REF_TAU 4
#endif

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

struct Int32Rates stabilization_rate_sp;
struct Int32Rates stabilization_rate_gain;
struct Int32Rates stabilization_rate_igain;
struct Int32Rates stabilization_rate_ddgain;
struct Int32Rates stabilization_rate_ref;
struct Int32Rates stabilization_rate_refdot;
struct Int32Rates stabilization_rate_sum_err;

struct Int32Rates stabilization_rate_fb_cmd;
struct Int32Rates stabilization_rate_ff_cmd;


#ifndef STABILIZATION_RATE_DEADBAND_P
#define STABILIZATION_RATE_DEADBAND_P 0
#endif
#ifndef STABILIZATION_RATE_DEADBAND_Q
#define STABILIZATION_RATE_DEADBAND_Q 0
#endif
#ifndef STABILIZATION_RATE_DEADBAND_R
#define STABILIZATION_RATE_DEADBAND_R 200
#endif

#define ROLL_RATE_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_ROLL] >  STABILIZATION_RATE_DEADBAND_P || \
   radio_control.values[RADIO_ROLL] < -STABILIZATION_RATE_DEADBAND_P)

#define PITCH_RATE_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_PITCH] >  STABILIZATION_RATE_DEADBAND_Q || \
   radio_control.values[RADIO_PITCH] < -STABILIZATION_RATE_DEADBAND_Q)

#define YAW_RATE_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_YAW] >  STABILIZATION_RATE_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_RATE_DEADBAND_R)

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_rate(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_RATE_LOOP(trans, dev, AC_ID,
                          &stabilization_rate_sp.p,
                          &stabilization_rate_sp.q,
                          &stabilization_rate_sp.r,
                          &stabilization_rate_ref.p,
                          &stabilization_rate_ref.q,
                          &stabilization_rate_ref.r,
                          &stabilization_rate_refdot.p,
                          &stabilization_rate_refdot.q,
                          &stabilization_rate_refdot.r,
                          &stabilization_rate_sum_err.p,
                          &stabilization_rate_sum_err.q,
                          &stabilization_rate_sum_err.r,
                          &stabilization_rate_ff_cmd.p,
                          &stabilization_rate_ff_cmd.q,
                          &stabilization_rate_ff_cmd.r,
                          &stabilization_rate_fb_cmd.p,
                          &stabilization_rate_fb_cmd.q,
                          &stabilization_rate_fb_cmd.r,
                          &stabilization_cmd[COMMAND_THRUST]);
}
#endif

void stabilization_rate_init(void)
{

  INT_RATES_ZERO(stabilization_rate_sp);

  RATES_ASSIGN(stabilization_rate_gain,
               STABILIZATION_RATE_GAIN_P,
               STABILIZATION_RATE_GAIN_Q,
               STABILIZATION_RATE_GAIN_R);
  RATES_ASSIGN(stabilization_rate_igain,
               STABILIZATION_RATE_IGAIN_P,
               STABILIZATION_RATE_IGAIN_Q,
               STABILIZATION_RATE_IGAIN_R);
  RATES_ASSIGN(stabilization_rate_ddgain,
               STABILIZATION_RATE_DDGAIN_P,
               STABILIZATION_RATE_DDGAIN_Q,
               STABILIZATION_RATE_DDGAIN_R);

  INT_RATES_ZERO(stabilization_rate_ref);
  INT_RATES_ZERO(stabilization_rate_refdot);
  INT_RATES_ZERO(stabilization_rate_sum_err);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "RATE_LOOP", send_rate);
#endif
}


void stabilization_rate_read_rc(void)
{

  if (ROLL_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.p = (int32_t)radio_control.values[RADIO_ROLL] * STABILIZATION_RATE_SP_MAX_P / MAX_PPRZ;
  } else {
    stabilization_rate_sp.p = 0;
  }

  if (PITCH_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.q = (int32_t)radio_control.values[RADIO_PITCH] * STABILIZATION_RATE_SP_MAX_Q / MAX_PPRZ;
  } else {
    stabilization_rate_sp.q = 0;
  }

  if (YAW_RATE_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
    stabilization_rate_sp.r = (int32_t)radio_control.values[RADIO_YAW] * STABILIZATION_RATE_SP_MAX_R / MAX_PPRZ;
  } else {
    stabilization_rate_sp.r = 0;
  }

  // Setpoint at ref resolution
  INT_RATES_LSHIFT(stabilization_rate_sp, stabilization_rate_sp, REF_FRAC - INT32_RATE_FRAC);
}

//Read rc with roll and yaw sitcks switched if the default orientation is vertical but airplane sticks are desired
void stabilization_rate_read_rc_switched_sticks(void)
{

  if (ROLL_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.r = (int32_t) - radio_control.values[RADIO_ROLL] * STABILIZATION_RATE_SP_MAX_P / MAX_PPRZ;
  } else {
    stabilization_rate_sp.r = 0;
  }

  if (PITCH_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.q = (int32_t)radio_control.values[RADIO_PITCH] * STABILIZATION_RATE_SP_MAX_Q / MAX_PPRZ;
  } else {
    stabilization_rate_sp.q = 0;
  }

  if (YAW_RATE_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
    stabilization_rate_sp.p = (int32_t)radio_control.values[RADIO_YAW] * STABILIZATION_RATE_SP_MAX_R / MAX_PPRZ;
  } else {
    stabilization_rate_sp.p = 0;
  }

  // Setpoint at ref resolution
  INT_RATES_LSHIFT(stabilization_rate_sp, stabilization_rate_sp, REF_FRAC - INT32_RATE_FRAC);
}

void stabilization_rate_enter(void)
{
  RATES_COPY(stabilization_rate_ref, stabilization_rate_sp);
  INT_RATES_ZERO(stabilization_rate_sum_err);
}

void stabilization_rate_run(bool_t in_flight)
{

  /* reference */
  struct Int32Rates _r;
  RATES_DIFF(_r, stabilization_rate_sp, stabilization_rate_ref);
  RATES_SDIV(stabilization_rate_refdot, _r, STABILIZATION_RATE_REF_TAU);
  /* integrate ref */
  const struct Int32Rates _delta_ref = {
    stabilization_rate_refdot.p >> (F_UPDATE_RES + REF_DOT_FRAC - REF_FRAC),
    stabilization_rate_refdot.q >> (F_UPDATE_RES + REF_DOT_FRAC - REF_FRAC),
    stabilization_rate_refdot.r >> (F_UPDATE_RES + REF_DOT_FRAC - REF_FRAC)
  };
  RATES_ADD(stabilization_rate_ref, _delta_ref);

  /* compute feed-forward command */
  RATES_EWMULT_RSHIFT(stabilization_rate_ff_cmd, stabilization_rate_ddgain, stabilization_rate_refdot, 9);


  /* compute feed-back command */
  /* error for feedback */
  const struct Int32Rates _ref_scaled = {
    OFFSET_AND_ROUND(stabilization_rate_ref.p, (REF_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stabilization_rate_ref.q, (REF_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stabilization_rate_ref.r, (REF_FRAC - INT32_RATE_FRAC))
  };
  struct Int32Rates _error;
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  RATES_DIFF(_error, _ref_scaled, (*body_rate));
  if (in_flight) {
    /* update integrator */
    RATES_ADD(stabilization_rate_sum_err, _error);
    RATES_BOUND_CUBE(stabilization_rate_sum_err, -MAX_SUM_ERR, MAX_SUM_ERR);
  } else {
    INT_RATES_ZERO(stabilization_rate_sum_err);
  }

  /* PI */
  stabilization_rate_fb_cmd.p = stabilization_rate_gain.p * _error.p +
                                OFFSET_AND_ROUND2((stabilization_rate_igain.p  * stabilization_rate_sum_err.p), 10);

  stabilization_rate_fb_cmd.q = stabilization_rate_gain.q * _error.q +
                                OFFSET_AND_ROUND2((stabilization_rate_igain.q  * stabilization_rate_sum_err.q), 10);

  stabilization_rate_fb_cmd.r = stabilization_rate_gain.r * _error.r +
                                OFFSET_AND_ROUND2((stabilization_rate_igain.r  * stabilization_rate_sum_err.r), 10);

  stabilization_rate_fb_cmd.p = stabilization_rate_fb_cmd.p >> 11;
  stabilization_rate_fb_cmd.q = stabilization_rate_fb_cmd.q >> 11;
  stabilization_rate_fb_cmd.r = stabilization_rate_fb_cmd.r >> 11;

  /* sum to final command */
  stabilization_cmd[COMMAND_ROLL]  = stabilization_rate_ff_cmd.p + stabilization_rate_fb_cmd.p;
  stabilization_cmd[COMMAND_PITCH] = stabilization_rate_ff_cmd.q + stabilization_rate_fb_cmd.q;
  stabilization_cmd[COMMAND_YAW]   = stabilization_rate_ff_cmd.r + stabilization_rate_fb_cmd.r;

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);

}
