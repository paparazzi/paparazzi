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
 * @file modules/guidance_opticflow/guidance_opticflow_hover.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

// Own Header
#include "guidance_opticflow_hover.h"

#include "modules/core/abi.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "modules/datalink/downlink.h"

/** Default sender to accect VELOCITY_ESTIMATE messages from */
#ifndef VISION_VELOCITY_ESTIMATE_ID
#define VISION_VELOCITY_ESTIMATE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VISION_VELOCITY_ESTIMATE_ID)

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN)

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN)

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN)

#ifndef VISION_DESIRED_VX
#define VISION_DESIRED_VX 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VX)

#ifndef VISION_DESIRED_VY
#define VISION_DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VY)

/* Check the control gains */
#if (VISION_PHI_PGAIN < 0)      ||  \
  (VISION_PHI_IGAIN < 0)        ||  \
  (VISION_THETA_PGAIN < 0)      ||  \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

static abi_event velocity_est_ev;

/* Initialize the default gains and settings */
struct opticflow_stab_t opticflow_stab = {
  .phi_pgain = VISION_PHI_PGAIN,
  .phi_igain = VISION_PHI_IGAIN,
  .theta_pgain = VISION_THETA_PGAIN,
  .theta_igain = VISION_THETA_IGAIN,
  .desired_vx = VISION_DESIRED_VX,
  .desired_vy = VISION_DESIRED_VY
};


static void stabilization_opticflow_vel_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp, float vel_x, float vel_y, float vel_z, float noise_x, float noise_y, float noise_z);
/**
 * Initialization of horizontal guidance module.
 */
void guidance_opticflow_hover_init(void)
{
  // Subscribe to the VELOCITY_ESTIMATE ABI message
  AbiBindMsgVELOCITY_ESTIMATE(VISION_VELOCITY_ESTIMATE_ID, &velocity_est_ev, stabilization_opticflow_vel_cb);
}

/**
 * guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_module_enter(void)
{
  /* Reset the integrated errors */
  opticflow_stab.err_vx_int = 0;
  opticflow_stab.err_vy_int = 0;

  /* Set rool/pitch to 0 degrees and psi to current heading */
  opticflow_stab.cmd.phi = 0;
  opticflow_stab.cmd.theta = 0;
  opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_module_run(bool in_flight)
{
  struct StabilizationSetpoint sp = stab_sp_from_eulers_i(&opticflow_stab.cmd);
  struct ThrustSetpoint th = guidance_v_run(in_flight);
  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight, &sp, &th, stabilization.cmd);
}

/**
 * Update the controls on a new VELOCITY_ESTIMATE ABI message.
 */
static void stabilization_opticflow_vel_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp UNUSED, float vel_x, float vel_y, float vel_z UNUSED,
    float noise_x, float noise_y, float noise_z UNUSED)
{
  if (noise_x >= 0.f)
  {
    /* Calculate the error */
    float err_vx = opticflow_stab.desired_vx - vel_x;

    /* Calculate the integrated errors (TODO: bound??) */
    opticflow_stab.err_vx_int += err_vx / 512;

    /* Calculate the commands */
    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vx
                               + opticflow_stab.theta_igain * opticflow_stab.err_vx_int);

    /* Bound the roll and pitch commands */
    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
  }

  if (noise_y >= 0.f)
  {
    /* Calculate the error */
    float err_vy = opticflow_stab.desired_vy - vel_y;

    /* Calculate the integrated errors (TODO: bound??) */
    opticflow_stab.err_vy_int += err_vy / 512;

    /* Calculate the commands */
    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vy
                               + opticflow_stab.phi_igain * opticflow_stab.err_vy_int;

    /* Bound the roll and pitch commands */
    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
  }
}
