/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/ctrl/control_mixing_heewing.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control mixing specific to the Heewing T1 Ranger
 */

#include "modules/ctrl/control_mixing_heewing.h"
#include "modules/radio_control/radio_control.h"
#include "generated/modules.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "autopilot.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/guidance/guidance_plane.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_plane_pid.h"

// Tilt position in forward flight
#ifndef CMH_TILT_FORWARD
#define CMH_TILT_FORWARD 0
#endif

// Tilt vertical position for hovering
#ifndef CMH_TILT_VERTICAL
#define CMH_TILT_VERTICAL 8700
#endif

// Max tilt differential for yaw
#ifndef CMH_TILT_DIFF_MAX
#define CMH_TILT_DIFF_MAX (MAX_PPRZ-CMH_TILT_VERTICAL)
#endif

// Motor idle position
#ifndef CMH_MOTOR_IDLE
#define CMH_MOTOR_IDLE 800
#endif

// Time for forward transition
#ifndef CMH_TRANSITION_TIME
#define CMH_TRANSITION_TIME 4.f
#endif

// Transition airspeed, set a negative value to disable
#ifndef CMH_TRANSITION_AIRSPEED
#define CMH_TRANSITION_AIRSPEED 10.f
#endif

#define TRANSITION_TO_HOVER false
#define TRANSITION_TO_FORWARD true

static float transition_ratio;
static const float transition_increment = 1.f / (CMH_TRANSITION_TIME * PERIODIC_FREQUENCY);

static void transition_run(bool to_forward) {
  if (to_forward && transition_ratio < 1.f) {
    if (stateIsAirspeedValid()) {
      if ((stateGetAirspeed_f() < CMH_TRANSITION_AIRSPEED && transition_ratio < 0.4f)
          || stateGetAirspeed_f() >= CMH_TRANSITION_AIRSPEED) {
        transition_ratio += transition_increment;
      }
      else if (stateGetAirspeed_f() < CMH_TRANSITION_AIRSPEED && transition_ratio > 0.4f) {
        transition_ratio -= transition_increment;
      }
    }
  } else if (!to_forward && transition_ratio > 0.f) {
    transition_ratio = 0.f; // immediately switch back to hover
  }
  Bound(transition_ratio, 0.f, 1.f);
}

static int32_t command_from_transition(int32_t hover_cmd, int32_t forward_cmd) {
  Bound(transition_ratio, 0.f, 1.f);
  return (int32_t) (hover_cmd + transition_ratio * (forward_cmd - hover_cmd));
}

void control_mixing_heewing_init(void)
{
  transition_ratio = 0.f; // 0. for hover to 1. for forward flight
}

void control_mixing_heewing_manual(void)
{
  transition_run(TRANSITION_TO_FORWARD);
  commands[COMMAND_ROLL] = radio_control_get(RADIO_ROLL);
  commands[COMMAND_PITCH] = radio_control_get(RADIO_PITCH);
  commands[COMMAND_YAW] = 0;
  commands[COMMAND_TILT] = CMH_TILT_FORWARD;
  commands[COMMAND_MOTOR_RIGHT] = radio_control_get(RADIO_THROTTLE);
  commands[COMMAND_MOTOR_LEFT] = radio_control_get(RADIO_THROTTLE);
  commands[COMMAND_MOTOR_TAIL] = MIN_PPRZ;
  commands[COMMAND_THRUST] = (commands[COMMAND_MOTOR_RIGHT] + commands[COMMAND_MOTOR_LEFT]) / 2;
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_heewing_attitude_direct(void)
{
  transition_run(TRANSITION_TO_HOVER);
  commands[COMMAND_TILT] = CMH_TILT_VERTICAL;
  commands[COMMAND_ROLL] = 0;
  commands[COMMAND_PITCH] = 0;
  struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());
  stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &th_sp, stabilization.cmd);
  if (autopilot_get_motors_on()) {
    commands[COMMAND_MOTOR_RIGHT] = actuators_pprz[CMH_ACT_MOTOR_RIGHT];
    commands[COMMAND_MOTOR_LEFT]  = actuators_pprz[CMH_ACT_MOTOR_LEFT];
    commands[COMMAND_MOTOR_TAIL]  = actuators_pprz[CMH_ACT_MOTOR_TAIL];
    if (autopilot_in_flight()) {
      commands[COMMAND_YAW]       = actuators_pprz[CMH_ACT_YAW];
    } else {
      commands[CMH_ACT_YAW]      = 0;
    }
    commands[COMMAND_THRUST]      = stabilization.cmd[COMMAND_THRUST];
  } else {
    commands[COMMAND_MOTOR_RIGHT] = -MAX_PPRZ;
    commands[COMMAND_MOTOR_LEFT]  = -MAX_PPRZ;
    commands[COMMAND_MOTOR_TAIL]  = -MAX_PPRZ;
    commands[COMMAND_YAW]         = 0;
    commands[COMMAND_THRUST]      = 0;
  }
  autopilot.throttle = commands[COMMAND_THRUST];
}

void control_mixing_heewing_attitude_direct_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_NONE, STABILIZATION_ATT_SUBMODE_HEADING); // force mode change to always reset heading
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void stabilization_indi_set_wls_settings(void)
{
   // Calculate the min and max increments
   wls_stab_p.u_min[CMH_ACT_MOTOR_RIGHT] = CMH_MOTOR_IDLE;
   wls_stab_p.u_max[CMH_ACT_MOTOR_RIGHT] = MAX_PPRZ;
   wls_stab_p.u_pref[CMH_ACT_MOTOR_RIGHT] = act_pref[CMH_ACT_MOTOR_RIGHT];

   wls_stab_p.u_min[CMH_ACT_MOTOR_LEFT] = CMH_MOTOR_IDLE;
   wls_stab_p.u_max[CMH_ACT_MOTOR_LEFT] = MAX_PPRZ;
   wls_stab_p.u_pref[CMH_ACT_MOTOR_LEFT] = act_pref[CMH_ACT_MOTOR_LEFT];

   wls_stab_p.u_min[CMH_ACT_MOTOR_TAIL] = CMH_MOTOR_IDLE;
   wls_stab_p.u_max[CMH_ACT_MOTOR_TAIL] = MAX_PPRZ;
   wls_stab_p.u_pref[CMH_ACT_MOTOR_TAIL] = act_pref[CMH_ACT_MOTOR_TAIL];

   wls_stab_p.u_min[CMH_ACT_YAW] = -CMH_TILT_DIFF_MAX;
   wls_stab_p.u_max[CMH_ACT_YAW] = CMH_TILT_DIFF_MAX;
   wls_stab_p.u_pref[CMH_ACT_YAW] = act_pref[CMH_ACT_YAW];
}

void control_mixing_heewing_attitude_plane_enter(void)
{
  // don't use forward submode to avoid pitch offset on RC input
  guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
  stabilization_attitude_plane_pid_enter();
}

void control_mixing_heewing_attitude_plane(void)
{
  transition_run(TRANSITION_TO_FORWARD);

  if (transition_ratio < 0.5f) {
    struct ThrustSetpoint hover_th_sp = guidance_v_run(autopilot_in_flight());
    stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &hover_th_sp, stabilization.cmd);
  }
  struct ThrustSetpoint th_sp = th_sp_from_thrust_i(radio_control_get(RADIO_THROTTLE), THRUST_AXIS_X);
  struct FloatEulers rc_sp = {
    .phi = STABILIZATION_ATTITUDE_SP_MAX_PHI * radio_control_get(RADIO_ROLL) / MAX_PPRZ,
    .theta = STABILIZATION_ATTITUDE_SP_MAX_THETA * radio_control_get(RADIO_PITCH) / MAX_PPRZ,
    .psi = 0.f
  };
  struct StabilizationSetpoint stab_sp = stab_sp_from_eulers_f(&rc_sp);
  stabilization_attitude_plane_pid_run(transition_ratio < 0.8 ? false : autopilot_in_flight(), &stab_sp, &th_sp, stabilization.cmd);

  commands[COMMAND_ROLL] = stabilization.cmd[COMMAND_ROLL];
  commands[COMMAND_PITCH] = stabilization.cmd[COMMAND_PITCH];
  commands[COMMAND_THRUST] = stabilization.cmd[COMMAND_THRUST];
  if (autopilot_in_flight()) {
    if (transition_ratio < 0.5f) {
      // only hover stabilization
      commands[COMMAND_MOTOR_RIGHT] = actuators_pprz[CMH_ACT_MOTOR_RIGHT];
      commands[COMMAND_MOTOR_LEFT] = actuators_pprz[CMH_ACT_MOTOR_LEFT];
      commands[COMMAND_MOTOR_TAIL] = actuators_pprz[CMH_ACT_MOTOR_TAIL];
    } else {
      // blend with plane control
      commands[COMMAND_MOTOR_RIGHT] = command_from_transition(actuators_pprz[CMH_ACT_MOTOR_RIGHT], stabilization.cmd[COMMAND_THRUST]);
      commands[COMMAND_MOTOR_LEFT] = command_from_transition(actuators_pprz[CMH_ACT_MOTOR_LEFT], stabilization.cmd[COMMAND_THRUST]);
      commands[COMMAND_MOTOR_TAIL] = command_from_transition(actuators_pprz[CMH_ACT_MOTOR_TAIL], 0);
    }
  } else {
    commands[COMMAND_MOTOR_RIGHT] = stabilization.cmd[COMMAND_THRUST];
    commands[COMMAND_MOTOR_LEFT] = stabilization.cmd[COMMAND_THRUST];
    commands[COMMAND_MOTOR_TAIL] = 0;
  }
  commands[COMMAND_TILT] = command_from_transition(CMH_TILT_VERTICAL, CMH_TILT_FORWARD);
  commands[COMMAND_YAW] = 0;
  autopilot.throttle = commands[COMMAND_THRUST];

}

void control_mixing_heewing_nav_enter(void)
{
  guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
  guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
  stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING);
}

void control_mixing_heewing_nav_run(void)
{
  if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_ROUTE ||
      nav.horizontal_mode == NAV_HORIZONTAL_MODE_CIRCLE) {
    transition_run(TRANSITION_TO_FORWARD);

    struct ThrustSetpoint th_sp = guidance_plane_thrust_from_nav(transition_ratio < 0.8f ? false : autopilot_in_flight());
    if (transition_ratio < 0.8f) {
      guidance_plane.pitch_cmd = 0.f;
    }
    struct StabilizationSetpoint stab_sp = guidance_plane_attitude_from_nav(autopilot_in_flight());
    if (transition_ratio < 0.5f) {
      stabilization_attitude_plane_pid_run(transition_ratio < 0.8f ? false : autopilot_in_flight(), &stab_sp, &th_sp, stabilization.cmd);
      struct ThrustSetpoint hover_th_sp = guidance_v_run(autopilot_in_flight());
      struct FloatEulers eulers_sp = { .phi = 0.f , .theta = 0.f, .psi = stateGetNedToBodyEulers_f()->psi };
      struct StabilizationSetpoint hover_stab_sp = stab_sp_from_eulers_f(&eulers_sp);
      stabilization_run(autopilot_in_flight(), &hover_stab_sp, &hover_th_sp, stabilization.cmd); // will overwrite COMMAND_THRUST
    } else {
      stabilization_attitude_plane_pid_run(transition_ratio < 0.8f ? false : autopilot_in_flight(), &stab_sp, &th_sp, stabilization.cmd);
    }
    nav.heading = stateGetNedToBodyEulers_f()->psi; // overwrite nav heading to avoid problems when transition to hover

    commands[COMMAND_TILT] = command_from_transition(CMH_TILT_VERTICAL, CMH_TILT_FORWARD);
    commands[COMMAND_ROLL] = stabilization.cmd[COMMAND_ROLL];
    commands[COMMAND_PITCH] = stabilization.cmd[COMMAND_PITCH];
    commands[COMMAND_YAW] = command_from_transition(actuators_pprz[CMH_ACT_YAW], 0);
    if (autopilot_get_motors_on()) {
      if (transition_ratio < 0.5f) {
        // only hover stabilization
        commands[COMMAND_MOTOR_RIGHT] = actuators_pprz[CMH_ACT_MOTOR_RIGHT];
        commands[COMMAND_MOTOR_LEFT] = actuators_pprz[CMH_ACT_MOTOR_LEFT];
        commands[COMMAND_MOTOR_TAIL] = actuators_pprz[CMH_ACT_MOTOR_TAIL];
      } else {
        // blend with plane control
        commands[COMMAND_MOTOR_RIGHT] = command_from_transition(actuators_pprz[CMH_ACT_MOTOR_RIGHT], stabilization.cmd[COMMAND_THRUST]);
        commands[COMMAND_MOTOR_LEFT] = command_from_transition(actuators_pprz[CMH_ACT_MOTOR_LEFT], stabilization.cmd[COMMAND_THRUST]);
        commands[COMMAND_MOTOR_TAIL] = command_from_transition(actuators_pprz[CMH_ACT_MOTOR_TAIL], 0);
      }
      commands[COMMAND_THRUST] = stabilization.cmd[COMMAND_THRUST];
    } else {
      commands[COMMAND_THRUST]      = 0;
      commands[COMMAND_MOTOR_RIGHT] = MIN_PPRZ;
      commands[COMMAND_MOTOR_LEFT]  = MIN_PPRZ;
      commands[COMMAND_MOTOR_TAIL] = 0;
    }

  } else {
    // all other nav modes are in rotorcraft flight mode
    transition_run(TRANSITION_TO_HOVER);
    commands[COMMAND_TILT] = command_from_transition(CMH_TILT_VERTICAL, CMH_TILT_FORWARD);
    commands[COMMAND_ROLL] = 0;
    commands[COMMAND_PITCH] = 0;

    struct ThrustSetpoint th_sp = guidance_v_run(autopilot_in_flight());
    struct StabilizationSetpoint stab_sp = guidance_h_run(autopilot_in_flight());
    stabilization_run(autopilot_in_flight(), &stab_sp, &th_sp, stabilization.cmd);

    if (autopilot_get_motors_on()) {
      commands[COMMAND_MOTOR_RIGHT] = actuators_pprz[CMH_ACT_MOTOR_RIGHT];
      commands[COMMAND_MOTOR_LEFT]  = actuators_pprz[CMH_ACT_MOTOR_LEFT];
      commands[COMMAND_MOTOR_TAIL]  = command_from_transition(actuators_pprz[CMH_ACT_MOTOR_TAIL], 0);
      if (autopilot_in_flight()) {
        commands[COMMAND_YAW]       = actuators_pprz[CMH_ACT_YAW];
      } else {
        commands[CMH_ACT_YAW]       = 0;
      }
      commands[COMMAND_THRUST]      = stabilization.cmd[COMMAND_THRUST];
    } else {
      commands[COMMAND_MOTOR_RIGHT] = -MAX_PPRZ;
      commands[COMMAND_MOTOR_LEFT]  = -MAX_PPRZ;
      commands[COMMAND_MOTOR_TAIL]  = -MAX_PPRZ;
      commands[COMMAND_YAW]         = 0;
      commands[COMMAND_THRUST]      = 0;
    }
  }
  autopilot.throttle = commands[COMMAND_THRUST];
}
