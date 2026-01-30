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

#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

// proportional control part (horizontal position)
static float Ap = GUIDANCE_INDI_HINF_Ap;
static float Bp = GUIDANCE_INDI_HINF_Bp;
static float Cp = GUIDANCE_INDI_HINF_Cp;
static float Dp = GUIDANCE_INDI_HINF_Dp;
// derivative control part (horizontal speed)
static float Ad = GUIDANCE_INDI_HINF_Ad;
static float Bd = GUIDANCE_INDI_HINF_Bd;
static float Cd = GUIDANCE_INDI_HINF_Cd;
static float Dd = GUIDANCE_INDI_HINF_Dd;
// proportional control part (vertical position)
static float Apz = GUIDANCE_INDI_HINF_Apz;
static float Bpz = GUIDANCE_INDI_HINF_Bpz;
static float Cpz = GUIDANCE_INDI_HINF_Cpz;
static float Dpz = GUIDANCE_INDI_HINF_Dpz;
// derivative control part (vertical speed)
static float Adz = GUIDANCE_INDI_HINF_Adz;
static float Bdz = GUIDANCE_INDI_HINF_Bdz;
static float Cdz = GUIDANCE_INDI_HINF_Cdz;
static float Ddz = GUIDANCE_INDI_HINF_Ddz;

static struct FloatVect3 pos_state = { 0 };
static struct FloatVect3 speed_state = { 0 };

/** Acceleration controller based Hinfinity
 */
struct FloatVect3 guidance_indi_controller(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode)
{
  struct FloatVect3 pos_err = { 0 };
  struct FloatVect3 speed_err = { 0 };

  struct FloatVect3 accel_sp = { 0 };
  struct FloatVect3 speed_sp = { 0 };
  struct FloatVect3 speed_fb = { 0 };

  if (!in_flight) {
    FLOAT_VECT3_ZERO(pos_state);
    FLOAT_VECT3_ZERO(speed_state);
    // TODO return with no control ?
  }

  if (h_mode == GUIDANCE_INDI_H_ACCEL) {
    // Speed feedback is included in the guidance when running in ACCEL mode
    speed_fb.x = 0.f;
    speed_fb.y = 0.f;
  }
  else {
    // Generate speed feedback for acceleration, as it is estimated
    if (h_mode == GUIDANCE_INDI_H_SPEED) {
      speed_sp.x = SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
      speed_sp.y = SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    }
    else {
      // H_POS
      // speed setpoint from position error
      pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
      pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
      speed_sp.x = Cp * pos_state.x + Dp * pos_err.x;
      speed_sp.y = Cp * pos_state.y + Dp * pos_err.y;
      pos_state.x = Ap * pos_state.x + Bd * pos_err.x;
      pos_state.y = Ap * pos_state.y + Bp * pos_err.y;

      // TODO where should we add the feed-forward ref speed ?
      //speed_sp.x += SPEED_FLOAT_OF_BFP(gh->ref.speed.x);
      //speed_sp.y += SPEED_FLOAT_OF_BFP(gh->ref.speed.y);
    }
    speed_err.x = speed_sp.x - stateGetSpeedNed_f()->x;
    speed_err.y = speed_sp.y - stateGetSpeedNed_f()->y;
    speed_fb.x = Cd * speed_state.x + Dd * speed_err.x;
    speed_fb.y = Cd * speed_state.y + Dd * speed_err.y;
    speed_state.x = Ad * speed_state.x + Bd * speed_err.x;
    speed_state.y = Ad * speed_state.y + Bd * speed_err.y;
  }

  if (v_mode == GUIDANCE_INDI_V_ACCEL)  {
    // Speed feedback is included in the guidance when running in ACCEL mode
    speed_fb.z = 0;
  }
  else {
    // Generate speed feedback for acceleration, as it is estimated
    if (v_mode == GUIDANCE_INDI_V_SPEED) {
      speed_sp.z = SPEED_FLOAT_OF_BFP(gv->zd_ref);
    }
    else {
      // V_POS
      // vertical speed setpoint from altitude error
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      speed_sp.z = Cpz * pos_state.z + Dpz * pos_err.z;
      pos_state.z = Apz * pos_state.z + Bpz * pos_err.z;
      // TODO add speed feed-forward ?
      // speed_sp.z += SPEED_FLOAT_OF_BFP(gv->zd_ref);
    }
    speed_err.z = speed_sp.z - stateGetSpeedNed_f()->z;
    speed_fb.z = Cdz * speed_state.z + Ddz * speed_err.z;
    speed_state.z = Adz * speed_state.z + Bdz * speed_err.z;
  }

  // TODO add accel feed-forward term ?
  accel_sp.x = speed_fb.x; // + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
  accel_sp.y = speed_fb.y; // + ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
  accel_sp.z = speed_fb.z; // + ACCEL_FLOAT_OF_BFP(gv->zdd_ref);

  return accel_sp;
}


