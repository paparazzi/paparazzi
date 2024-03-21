/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/ctrl_module_outerloop_demo.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/ctrl_module_outerloop_demo.h"
#include "state.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "modules/radio_control/radio_control.h"
#include "autopilot.h"

// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct AttitudeRCInput rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;


// Settings
float comode_time = 0;


////////////////////////////////////////////////////////////////////
// Call our controller
void ctrl_module_init(void)
{
  stabilization_attitude_rc_setpoint_init(&ctrl.rc_sp);
}

void guidance_module_enter(void)
{
  // Store current heading
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);

  // vertical mode in hover
  guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
}

void guidance_module_run(bool in_flight)
{
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);

  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);
  float roll = 0.0;
  float pitch = 0.0;

  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);

  struct StabilizationSetpoint sp = stab_sp_from_eulers_i(&(ctrl.cmd));
  struct ThrustSetpoint th = guidance_v_run(in_flight);
  // execute attitude stabilization:
  stabilization_attitude_run(in_flight, &sp, &th, stabilization.cmd);
}

