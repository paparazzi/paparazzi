/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *
 * This file is part of paparazzi
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

/** @file "modules/ctrl/eff_scheduling_nederdrone.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Interpolation of control effectivenss matrix
of the Nederdrone.

If instead using online adaptation is an option, be sure to
not use this module at the same time!
 */

#include "modules/ctrl/eff_scheduling_nederdrone.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "state.h"
#include "autopilot.h"
#include "modules/radio_control/radio_control.h"
#include "generated/radio.h"
#include "modules/actuators/actuators.h"
#define INDI_SCHEDULING_LOWER_BOUND_G1 0.0001

// Airspeed at which tip props should be turned on
#ifndef INDI_SCHEDULING_LOW_AIRSPEED
#define INDI_SCHEDULING_LOW_AIRSPEED 12.0
#endif

#if INDI_NUM_ACT < 8
#error "This module works with a Nederdrone with 8 (grouped) actuators"
#endif

#ifndef INDI_SCHEDULING_TRIM_ELEVATOR
#define INDI_SCHEDULING_TRIM_ELEVATOR 0.0
#endif

#ifndef INDI_SCHEDULING_TRIM_FLAPS
#define INDI_SCHEDULING_TRIM_FLAPS 0.0
#endif

float trim_elevator = INDI_SCHEDULING_TRIM_ELEVATOR;
float trim_flaps = INDI_SCHEDULING_TRIM_FLAPS;

bool all_act_fwd_sched = false;

int32_t use_scheduling = 1;

float thrust_eff_scaling = 1.0;

static float g_forward[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL_FWD, STABILIZATION_INDI_G1_PITCH_FWD, STABILIZATION_INDI_G1_YAW_FWD, STABILIZATION_INDI_G1_THRUST_FWD};

static float g_hover[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL, STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST};

// Functions to schedule switching on and of of tip props on front wing
float sched_ratio_tip_props = 1.0;
// If pitch lower, pitch props gradually switch off till  sched_tip_prop_lower_pitch_limit_deg (1 > sched_ratio_tip_props > 0)
float sched_tip_prop_upper_pitch_limit_deg = -45;
// If pitch lower, pitch props switch fully off (sched_ratio_tip_props goes to 0)
float sched_tip_prop_lower_pitch_limit_deg = -65;
// Setting to not switch off tip props during forward flight
bool sched_tip_props_always_on = false;

void schdule_control_effectiveness(void);

void ctrl_eff_scheduling_init(void)
{
  // your init code here
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    g_hover[0][i] = g_hover[0][i] / INDI_G_SCALING;
    g_hover[1][i] = g_hover[1][i] / INDI_G_SCALING;
    g_hover[2][i] = g_hover[2][i] / INDI_G_SCALING;
    g_hover[3][i] = g_hover[3][i] / INDI_G_SCALING;

    g_forward[0][i] = g_forward[0][i] / INDI_G_SCALING;
    g_forward[1][i] = g_forward[1][i] / INDI_G_SCALING;
    g_forward[2][i] = g_forward[2][i] / INDI_G_SCALING;
    g_forward[3][i] = g_forward[3][i] / INDI_G_SCALING;
  }
}

void ctrl_eff_scheduling_periodic(void)
{
#ifdef SITL
  sched_ratio_tip_props = 1.0;
#else
  schdule_control_effectiveness();
#endif

  act_pref[0] = 0.0;
  act_pref[1] = 0.0;
  act_pref[2] = 0.0;
  act_pref[3] = 0.0;
  // read settings and trim the aerodynamic surfaces
  act_pref[4] = -trim_elevator - trim_flaps;
  act_pref[5] = -trim_elevator - trim_flaps;
  act_pref[6] = -trim_elevator + trim_flaps;
  act_pref[7] = -trim_elevator + trim_flaps;
}

/**
 * Function that calculates control effectiveness values for the Nederdrone inner loop.
 * Default requency: 20 Hz.
 */
void schdule_control_effectiveness(void) {
  float airspeed = stateGetAirspeed_f();

  float ratio = 0.0;
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  // Ratio is only based on pitch now, as the pitot tube is often not mounted.
  if (use_scheduling == 1) {
    ratio = fabs(eulers_zxy.theta) / M_PI_2;
  } else {
    ratio = 0.0;
  }
  Bound(ratio,0.0,1.0);

  float stall_speed = 14.0; // m/s
  float pitch_ratio = 0.0;
  // Assume hover or stalled conditions below 14 m/s
  if (use_scheduling == 1) {
    if ( (eulers_zxy.theta > -M_PI_4) || (airspeed < stall_speed) ) {
      if (eulers_zxy.theta > -M_PI_4) {
        pitch_ratio = 0.0;
      } else {
        pitch_ratio = fabs(1.0 - eulers_zxy.theta/(-M_PI_4));
      }

    } else {
      pitch_ratio = 1.0;
    }
  } else {
    pitch_ratio = 0.0;
  }
  Bound(pitch_ratio,0.0,1.0);

  float airspeed_pitch_eff = airspeed;
  Bound(airspeed_pitch_eff, 8.0, 30.0);

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {

    // Roll
    g1g2[0][i] = g_hover[0][i] * (1.0 - ratio) + g_forward[0][i] * ratio;
    //Pitch, scaled with v^2
    if (i>3 || all_act_fwd_sched) {
      g1g2[1][i] = g_hover[1][i] * (1.0 - pitch_ratio) + g_forward[1][i] * pitch_ratio;
    } else {
      g1g2[1][i] = g_hover[1][i] * (1.0 - pitch_ratio) + g_forward[1][i] * pitch_ratio * airspeed_pitch_eff * airspeed_pitch_eff / (16.0*16.0);
    }
    //Yaw
    g1g2[2][i] = g_hover[2][i] * (1.0 - ratio) + g_forward[2][i] * ratio;

    // Determine thrust of the wing to adjust the effectiveness of servos
    float wing_thrust_scaling = 1.0;
#if INDI_NUM_ACT != 8
#error "ctfl_eff_scheduling_nederdrone is very specific and only works for one Nederdrone configuration!"
#endif
    if (i>3) {
      float wing_thrust = actuators_pprz[i-4];
      Bound(wing_thrust,3000.0,9600.0);
      wing_thrust_scaling = wing_thrust/9600.0/0.8;
    }

    g1g2[0][i] *= wing_thrust_scaling;
    g1g2[1][i] *= wing_thrust_scaling;
    g1g2[2][i] *= wing_thrust_scaling;
  }

  // Thrust effectiveness
  float ratio_spec_force = 0.0;
  float airspeed_spec_force = airspeed;
  Bound(airspeed_spec_force, 8.0, 20.0);
  ratio_spec_force = (airspeed_spec_force-8.0) / 12.0;

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    // Thrust
    g1g2[3][i] = g_hover[3][i] * (1.0 - ratio_spec_force) + g_forward[3][i] * ratio_spec_force;
    g1g2[3][i] *= thrust_eff_scaling;
  }

  bool low_airspeed = stateGetAirspeed_f() < INDI_SCHEDULING_LOW_AIRSPEED;

  // Tip prop ratio
  float pitch_deg = eulers_zxy.theta / M_PI * 180.f;
  float pitch_range_deg = sched_tip_prop_upper_pitch_limit_deg - sched_tip_prop_lower_pitch_limit_deg;
  if (sched_tip_props_always_on || low_airspeed || radio_control.values[RADIO_AUX2] > 0) {
    sched_ratio_tip_props = 1.0;
  } else {
    float pitch_offset = pitch_deg - sched_tip_prop_lower_pitch_limit_deg;
    sched_ratio_tip_props = pitch_offset / pitch_range_deg;
  }
  Bound(sched_ratio_tip_props, 0.0, 1.0);
}
