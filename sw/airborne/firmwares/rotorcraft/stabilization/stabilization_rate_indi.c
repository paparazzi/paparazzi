/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2020 Rohan Chotalal
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

/** @file stabilization_rate_indi.c
 *  Rate stabilization for rotorcrafts based on INDI by Ewoud Smeur.
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_indi.h"

#ifdef STABILIZATION_ATTITUDE_INDI_SIMPLE
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#else
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

#include "firmwares/rotorcraft/autopilot_rc_helpers.h"

/* -- define variables */
struct FloatRates stabilization_rate_sp;

/** Maximum rate you can request in RC rate mode (rad/s)*/
#ifndef STABILIZATION_INDI_MAX_RATE
#define STABILIZATION_INDI_MAX_RATE 6.0
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_rate(struct transport_tx *trans, struct link_device *dev)
{
  float dummy = 0;
  float fb_p = stabilization.cmd[COMMAND_ROLL];
  float fb_q = stabilization.cmd[COMMAND_PITCH];
  float fb_r = stabilization.cmd[COMMAND_YAW];

  pprz_msg_send_RATE_LOOP(trans, dev, AC_ID,
                          &stabilization_rate_sp.p,
                          &stabilization_rate_sp.q,
                          &stabilization_rate_sp.r,
                          &dummy, &dummy, &dummy,
                          &fb_p,
                          &fb_q,
                          &fb_r,
                          &stabilization.cmd[COMMAND_THRUST]);
}
#endif

/**
 * @brief Initialize rate controller
 */
void stabilization_rate_init(void)
{
  FLOAT_RATES_ZERO(stabilization_rate_sp);
  // indi init is already done through module init

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RATE_LOOP, send_rate);
#endif
}

/**
 * @brief Reset rate controller
 */
void stabilization_rate_enter(void)
{
  stabilization_indi_enter();
}

/**
 * @brief Run indi rate interface from the "stabilization_rate_run" function
 */
void stabilization_rate_run(bool in_flight, struct StabilizationSetpoint *rate_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stabilization_rate_sp = stab_sp_to_rates_f(rate_sp);

  /* compute the INDI rate command */
  stabilization_indi_rate_run(in_flight, rate_sp, thrust, cmd);
}

struct StabilizationSetpoint stabilization_rate_read_rc(struct RadioControl *rc)
{
  struct FloatRates rate_sp;
  FLOAT_RATES_ZERO(rate_sp);
  if (ROLL_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.p = rc->values[RC_RATE_P] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  }
  if (PITCH_RATE_DEADBAND_EXCEEDED(rc)) {
    rate_sp.q = rc->values[RC_RATE_Q] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  }
  if (YAW_RATE_DEADBAND_EXCEEDED(rc) && !THROTTLE_STICK_DOWN_FROM_RC(rc)) {
    rate_sp.r = rc->values[RC_RATE_R] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  }
  return stab_sp_from_rates_f(&rate_sp);
}

