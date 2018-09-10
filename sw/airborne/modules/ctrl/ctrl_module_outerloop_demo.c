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
 * @file modules/ctrl/ctrl_module_demo.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/ctrl_module_outerloop_demo.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

// Own Settings
float comode_time = 1.0;


// Own Variables

struct ctrl_module_demo_struct {
// Settings

// RC Inputs
  pprz_t rc_x;
  pprz_t rc_y;
  pprz_t rc_z;
  pprz_t rc_t;

// Output command
  struct Int32Eulers cmd;
} ctrl;



////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
}

void guidance_h_module_enter(void)
{
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  ctrl.rc_x = 0;
  ctrl.rc_y = 0;
}

void guidance_h_module_read_rc(void)
{
  // -MAX_PPRZ to MAX_PPRZ
  //ctrl.rc_t = radio_control.values[RADIO_THROTTLE];
  ctrl.rc_x = radio_control.values[RADIO_ROLL];
  ctrl.rc_y = radio_control.values[RADIO_PITCH];
  //ctrl.rc_z = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(bool in_flight)
{
  // Demo which copies the sticks into pitch and roll
  float roll = ctrl.rc_x;
  roll /= (float) MAX_PPRZ;
  roll *= RadOfDeg(10);

  float pitch = ctrl.rc_y;
  pitch /= (float) MAX_PPRZ;
  pitch *= RadOfDeg(10);

  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);

  ctrl.cmd.phi = roll;
  ctrl.cmd.theta = pitch;

  //int32_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));

  stabilization_attitude_run(in_flight);
}

