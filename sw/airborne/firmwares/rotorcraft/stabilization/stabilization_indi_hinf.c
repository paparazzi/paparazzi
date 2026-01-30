/*
 * Copyright (C) Gautier Hattenberger, Mohamad Hachem
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "math/pprz_algebra_float.h"
#include "autopilot.h"

static struct FloatRates rate_state;
static struct FloatRates att_state;

// proportional control part (attitude)
static float Ap = STABILIZATION_INDI_HINF_Ap;
static float Bp = STABILIZATION_INDI_HINF_Bp;
static float Cp = STABILIZATION_INDI_HINF_Cp;
static float Dp = STABILIZATION_INDI_HINF_Dp;
// derivative control part (rates)
static float Ad = STABILIZATION_INDI_HINF_Ad;
static float Bd = STABILIZATION_INDI_HINF_Bd;
static float Cd = STABILIZATION_INDI_HINF_Cd;
static float Dd = STABILIZATION_INDI_HINF_Dd;

/** Angular acceleration controller based on Hinfinity
 *
 * Takes the current rates filtered state and setpoint and compute the desired acceleration.
 */
struct FloatRates stabilization_indi_rate_controller(struct FloatRates rates, struct FloatRates sp)
{
  struct FloatRates rate_error;
  RATES_DIFF(rate_error, sp, rates);

  struct FloatRates accel_ref;
  if (autopilot_in_flight()) {
    accel_ref.p = Cd * rate_state.p + Dd * rate_error.p;
    rate_state.p = Ad * rate_state.p + Bd * rate_error.p;

    accel_ref.q = Cd * rate_state.q + Dd * rate_error.q;
    rate_state.q = Ad * rate_state.q + Bd * rate_error.q;

    accel_ref.r = rate_error.r * indi_gains.rate.r;
  } else {
    FLOAT_RATES_ZERO(rate_state);
    FLOAT_RATES_ZERO(accel_ref);
  }

  return accel_ref;
}

/** Angular rate controller based on Hinfinity
 *
 * Takes the current attitude filtered state and setpoint and compute the desired rates.
 * Can be redefined elsewhere to use an other control scheme.
 */
struct FloatRates WEAK stabilization_indi_attitude_controller(struct FloatQuat att, struct FloatQuat att_sp, struct FloatRates rates_ff)
{
  /* attitude error */
  struct FloatQuat att_err;
  float_quat_inv_comp_norm_shortest(&att_err, &att, &att_sp);

  struct FloatVect3 att_fb;
#if TILT_TWIST_CTRL
  struct FloatQuat tilt;
  struct FloatQuat twist;
  float_quat_tilt_twist(&tilt, &twist, &att_err);
  att_fb.x = tilt.qx;
  att_fb.y = tilt.qy;
  att_fb.z = twist.qz;
#else
  att_fb.x = att_err.qx;
  att_fb.y = att_err.qy;
  att_fb.z = att_err.qz;
#endif

  struct FloatRates rate_sp;
  if (autopilot_in_flight()) {
    rate_sp.p  = Cp * att_state.p + Dp * att_fb.x;
    att_state.p = Ap * att_state.p + Bp * att_fb.x;

    rate_sp.q = Cp * att_state.q + Dp * att_fb.y;
    att_state.q = Ap * att_state.q + Bp * att_fb.y;

    rate_sp.r = indi_gains.att.r * att_fb.z / indi_gains.rate.r;

    // add feed-forward term
    RATES_ADD(rate_sp, rates_ff);
  } else {
    FLOAT_RATES_ZERO(att_state);
    FLOAT_RATES_ZERO(rate_sp);
  }

  return rate_sp;
}

