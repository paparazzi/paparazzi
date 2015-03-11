/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/opticflow/hover_stabilization.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

// Own Header
#include "stabilization_opticflow.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 500
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN);

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 10
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN);

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 500
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN);

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 10
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN);

#ifndef VISION_DESIRED_VX
#define VISION_DESIRED_VX 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VX);

#ifndef VISION_DESIRED_VY
#define VISION_DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VY);

/* Check the control gains */
#if (VISION_PHI_PGAIN < 0)      ||  \
  (VISION_PHI_IGAIN < 0)        ||  \
  (VISION_THETA_PGAIN < 0)      ||  \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct opticflow_stab_t opticflow_stab = {
  .phi_pgain = VISION_PHI_PGAIN,
  .phi_igain = VISION_PHI_IGAIN,
  .theta_pgain = VISION_THETA_PGAIN,
  .theta_igain = VISION_THETA_IGAIN,
  .desired_vx = VISION_DESIRED_VX,
  .desired_vy = VISION_DESIRED_VY
};

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  // Set the errors to 0
  opticflow_stab.err_vx = 0;
  opticflow_stab.err_vx_int = 0;
  opticflow_stab.err_vy = 0;
  opticflow_stab.err_vy_int = 0;

  // Set the euler command to 0
  INT_EULERS_ZERO(opticflow_stab.cmd);

  // GUIDANCE: Set Hover-z-hold (1 meter???)
  guidance_v_z_sp = -1;
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 */
void guidance_h_module_run(bool_t in_flight)
{
  // Run the default attitude stabilization
  stabilization_attitude_run(in_flight);
}

/**
 * Update the controls based on a vision result
 */
void stabilization_opticflow_update(struct opticflow_result_t* result)
{
  // *************************************************************************************
  // Downlink Message
  // *************************************************************************************

  float test = 1.5;
  DOWNLINK_SEND_OF_HOVER(DefaultChannel, DefaultDevice,
                         &result->fps, &result->dx_sum, &result->dy_sum, &result->OFx, &result->OFy,
                         &result->diff_roll, &result->diff_pitch,
                         &result->Velx, &result->Vely,
                         &test, &test,
                         &result->cam_h, &result->count);

  /* Check if we are in the correct AP_MODE before setting commands */
  if (autopilot_mode != AP_MODE_MODULE) {
    return;
  }

  /* Calculate the error if we have enough flow */
  if (result->flow_count > 0) {
    opticflow_stab.err_vx = result->Velx - opticflow_stab.desired_vx;
    opticflow_stab.err_vy = result->Vely - opticflow_stab.desired_vy;
  } else {
    opticflow_stab.err_vx = 0;
    opticflow_stab.err_vy = 0;
  }

  /* Calculate the integrated errors (TODO: bound??) */
  opticflow_stab.err_vx_int += opticflow_stab.theta_igain * opticflow_stab.err_vx;
  opticflow_stab.err_vy_int += opticflow_stab.phi_igain * opticflow_stab.err_vy;

  /* Calculate the commands */
  opticflow_stab.cmd.theta = (opticflow_stab.theta_pgain * opticflow_stab.err_vx + opticflow_stab.err_vx_int);
  opticflow_stab.cmd.phi = -(opticflow_stab.phi_pgain * opticflow_stab.err_vy + opticflow_stab.err_vy_int);

  /* Bound the roll and pitch commands */
  BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
  BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);

  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&opticflow_stab.cmd);
  //DOWNLINK_SEND_VISION_STABILIZATION(DefaultChannel, DefaultDevice, &result->Velx, &result->Vely, &Velx_Int,
  //                                   &Vely_Int, &cmd_euler.phi, &cmd_euler.theta);
}
