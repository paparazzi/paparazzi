/*
 * Copyright (C) 2014 Hann Woei Ho
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
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

// Own Header
#include "hover_stabilization.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"

// Downlink
#include "subsystems/datalink/downlink.h"

// Controller Gains
/* error if some gains are negative */
#if (VISION_PHI_PGAIN < 0)      ||                   \
  (VISION_PHI_IGAIN < 0)        ||                   \
  (VISION_THETA_PGAIN < 0)      ||                   \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif
bool activate_opticflow_hover;
float vision_desired_vx;
float vision_desired_vy;
int32_t vision_phi_pgain;
int32_t vision_phi_igain;
int32_t vision_theta_pgain;
int32_t vision_theta_igain;

// Controller Commands
struct Int32Eulers cmd_euler;

// Hover Stabilization
float Velx_Int;
float Vely_Int;
float Error_Velx;
float Error_Vely;

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851
unsigned char saturateX = 0, saturateY = 0;
unsigned int set_heading;


#ifndef VISION_HOVER
#define VISION_HOVER TRUE
#endif

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 500.
#endif

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 10.
#endif

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 500.
#endif

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 10.
#endif

#ifndef VISION_DESIRED_VX
#define VISION_DESIRED_VX 0.
#endif

#ifndef VISION_DESIRED_VY
#define VISION_DESIRED_VY 0.
#endif

void run_opticflow_hover(void);

void guidance_h_module_enter(void)
{
  // INIT
  Velx_Int = 0;
  Vely_Int = 0;
  // GUIDANCE: Set Hover-z-hold
  guidance_v_z_sp = -1;
}

void guidance_h_module_read_rc(void)
{
  // Do not read RC
  // Setpoint being set by vision
}

void guidance_h_module_run(bool_t in_flight)
{
  // Run
  // Setpoint being set by vision
  stabilization_attitude_run(in_flight);
}


void init_hover_stabilization_onvision()
{
  INT_EULERS_ZERO(cmd_euler);

  activate_opticflow_hover = VISION_HOVER;
  vision_phi_pgain = VISION_PHI_PGAIN;
  vision_phi_igain = VISION_PHI_IGAIN;
  vision_theta_pgain = VISION_THETA_PGAIN;
  vision_theta_igain = VISION_THETA_IGAIN;
  vision_desired_vx = VISION_DESIRED_VX;
  vision_desired_vy = VISION_DESIRED_VY;

  set_heading = 1;

  Error_Velx = 0;
  Error_Vely = 0;
  Velx_Int = 0;
  Vely_Int = 0;
}

void run_hover_stabilization_onvision(struct CVresults* vision )
{
  struct FloatVect3 V_body;
  if (activate_opticflow_hover == TRUE) {
    // Compute body velocities from ENU
    struct FloatVect3 *vel_ned = (struct FloatVect3*)stateGetSpeedNed_f();
    struct FloatQuat *q_n2b = stateGetNedToBodyQuat_f();
    float_quat_vmult(&V_body, q_n2b, vel_ned);
  }

  // *************************************************************************************
  // Downlink Message
  // *************************************************************************************

  DOWNLINK_SEND_OF_HOVER(DefaultChannel, DefaultDevice,
                         &vision->FPS, &vision->dx_sum, &vision->dy_sum, &vision->OFx, &vision->OFy,
                         &vision->diff_roll, &vision->diff_pitch,
                         &vision->Velx, &vision->Vely,
                         &V_body.x, &V_body.y,
                         &vision->cam_h, &vision->count);

  if (autopilot_mode != AP_MODE_MODULE) {
    return;
  }

  if (vision->flow_count) {
    Error_Velx = vision->Velx - vision_desired_vx;
    Error_Vely = vision->Vely - vision_desired_vy;
  } else {
    Error_Velx = 0;
    Error_Vely = 0;
  }

  if (saturateX == 0) {
    if (activate_opticflow_hover == TRUE) {
      Velx_Int += vision_theta_igain * Error_Velx;
    } else {
      Velx_Int += vision_theta_igain * V_body.x;
    }
  }
  if (saturateY == 0) {
    if (activate_opticflow_hover == TRUE) {
      Vely_Int += vision_phi_igain * Error_Vely;
    } else {
      Vely_Int += vision_phi_igain * V_body.y;
    }
  }

  if (set_heading) {
    cmd_euler.psi = stateGetNedToBodyEulers_i()->psi;
    set_heading = 0;
  }

  if (activate_opticflow_hover == TRUE) {
    cmd_euler.phi =  - (vision_phi_pgain * Error_Vely + Vely_Int);
    cmd_euler.theta = (vision_theta_pgain * Error_Velx + Velx_Int);
  } else {
    cmd_euler.phi =  - (vision_phi_pgain * V_body.y + Vely_Int);
    cmd_euler.theta = (vision_theta_pgain * V_body.x + Velx_Int);
  }

  saturateX = 0; saturateY = 0;
  if (cmd_euler.phi < -CMD_OF_SAT) {cmd_euler.phi = -CMD_OF_SAT; saturateX = 1;}
  else if (cmd_euler.phi > CMD_OF_SAT) {cmd_euler.phi = CMD_OF_SAT; saturateX = 1;}
  if (cmd_euler.theta < -CMD_OF_SAT) {cmd_euler.theta = -CMD_OF_SAT; saturateY = 1;}
  else if (cmd_euler.theta > CMD_OF_SAT) {cmd_euler.theta = CMD_OF_SAT; saturateY = 1;}

  stabilization_attitude_set_rpy_setpoint_i(&cmd_euler);
  DOWNLINK_SEND_VISION_STABILIZATION(DefaultChannel, DefaultDevice, &vision->Velx, &vision->Vely, &Velx_Int,
                                     &Vely_Int, &cmd_euler.phi, &cmd_euler.theta);
}
