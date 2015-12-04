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
#include "stabilization_practical.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"
#include "navigation.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851

#ifndef PRACTICAL_PHI_PGAIN
#define PRACTICAL_PHI_PGAIN 1000
#endif
PRINT_CONFIG_VAR(PRACTICAL_PHI_PGAIN);

#ifndef PRACTICAL_PHI_IGAIN
#define PRACTICAL_PHI_IGAIN 300
#endif
PRINT_CONFIG_VAR(PRACTICAL_PHI_IGAIN);

#ifndef PRACTICAL_THETA_PGAIN
#define PRACTICAL_THETA_PGAIN 1000
#endif
PRINT_CONFIG_VAR(PRACTICAL_THETA_PGAIN);
#ifndef PRACTICAL_THETA_IGAIN
#define PRACTICAL_THETA_IGAIN 300
#endif
PRINT_CONFIG_VAR(PRACTICAL_THETA_IGAIN);

#ifndef PRACTICAL_DESIRED_VX
#define PRACTICAL_DESIRED_VX 0.5
#endif
PRINT_CONFIG_VAR(PRACTICAL_DESIRED_VX);

#ifndef PRACTICAL_DESIRED_VY
#define PRACTICAL_DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(PRACTICAL_DESIRED_VY);

/* Check the control gains */
#if (PRACTICAL_PHI_PGAIN < 0)      ||  \
  (PRACTICAL_PHI_IGAIN < 0)        ||  \
  (PRACTICAL_THETA_PGAIN < 0)      ||  \
  (PRACTICAL_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct practical_stab_t practical_stab = {
  .phi_pgain = PRACTICAL_PHI_PGAIN,
  .phi_igain = PRACTICAL_PHI_IGAIN,
  .theta_pgain = PRACTICAL_THETA_PGAIN,
  .theta_igain = PRACTICAL_THETA_IGAIN,
  .desired_vx = PRACTICAL_DESIRED_VX,
  .desired_vy = PRACTICAL_DESIRED_VY
};

int32_t yaw_rate = 0;
float yaw_rate_write = 0;
float yaw_ref_write = 0;
int32_t keep_yaw_rate = 0;
int32_t keep_turning = 0;
float r_dot_new = 0;
float ref_pitch = 0.1;
float ref_roll = 0.1;
float speed_pot = 0;
float alpha_fil = 0.1;
float v_desired = 0.1;
float hopperdiepop = 4;

/*static void send_YAW_RATE(void) {
  DOWNLINK_SEND_YAW_RATE(DefaultChannel, DefaultDevice, &yaw_rate_write, &yaw_ref_write, &r_dot_new, &ref_pitch, &ref_roll);
 }
*/

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  /* Reset the integrated errors */
  practical_stab.err_vx_int = 0;
  practical_stab.err_vy_int = 0;

  /* Set rool/pitch to 0 degrees and psi to current heading */
  practical_stab.cmd.phi = 0;
  practical_stab.cmd.theta = 0;
  practical_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
  
  yaw_rate = 0;
  
  //register_periodic_telemetry(DefaultPeriodic, "YAW_RATE", send_YAW_RATE);
  
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
 * @param[in] in_flight Whether we are in flight or not
 */

/*Code from freek deleted out of run function 
 * 
     //Some logic to change the desired speed if outside boundery
    if(!InsideFlight_Area(GetPosX(), GetPosY())) {
      nav_set_heading_towards_waypoint(WP_MID); deleted by CN

      int32_t diff_heading = nav_heading - stateGetNedToBodyEulers_i()->psi;
      INT32_ANGLE_NORMALIZE(diff_heading);

      int32_t diff_lim_heading = diff_heading;
      BoundAbs(diff_lim_heading, 357*3); //15 deg
      practical_stab.cmd.psi =  stateGetNedToBodyEulers_i()->psi + diff_lim_heading;

      if(abs(diff_heading) < 357) { //5 deg
        practical_stab.desired_vx = PRACTICAL_DESIRED_VX;
        practical_stab.desired_vy = 0;
      } else {
        practical_stab.desired_vx = 0;
        practical_stab.desired_vy = 0;
      }
    }
    else {
      
      
    //printf("yaw_ref %d", practical_stab.cmd.psi);
    if(yaw_rate == 0) {
      if(keep_turning>0) {
	keep_turning = keep_turning - 1;
	practical_stab.cmd.psi += keep_yaw_rate;
      }
    }
    else{
      keep_yaw_rate = yaw_rate;
      keep_turning = 500/yaw_rate;
    }
      
    if(yaw_rate == 0){
      practical_stab.desired_vx = PRACTICAL_DESIRED_VX;
      //practical_stab.desired_vs = speed_pot;
    }
    else
      practical_stab.desired_vx = 0;
    //}     
      
 */

void guidance_h_module_run(bool_t in_flight)
{
  
    practical_stab.cmd.phi = ANGLE_BFP_OF_REAL(ref_roll);
    practical_stab.cmd.theta = ANGLE_BFP_OF_REAL(ref_pitch);
  

  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&practical_stab.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}