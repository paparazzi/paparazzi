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
 * @file modules/computer_vision/opticflow/stabilization_opticflow.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

// Own Header
#include "guidance_OA.h"

// Stabilization
#include <stdio.h>
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/electrical.h"
#include "std.h"
//#include "modules/read_matrix_serial/read_matrix_serial.h"

//#ifndef CMD_OF_SAT
#define CMD_OF_SAT  1500 // 1500 = 40 deg = 2859.1851
//#endif

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

/* Initialize the default gains and settings */
struct opticflow_stab_t opticflow_stab = {
  .phi_pgain = VISION_PHI_PGAIN,
  .phi_igain = VISION_PHI_IGAIN,
  .theta_pgain = VISION_THETA_PGAIN,
  .theta_igain = VISION_THETA_IGAIN,
  .desired_vx = VISION_DESIRED_VX,
  .desired_vy = VISION_DESIRED_VY
};

/////////Variables needed to set by user!!!!!/////
int8_t filter_flag = 0;    //0 =>no filter 1 =>Kalman filter 2 =>Butterworth filter
int8_t repulsionforce_filter_flag = 0;    //0 =>no filter 1 =>Butterworth filter

typedef enum {NO_OBSTACLE_AVOIDANCE,PINGPONG,POT_HEADING,POT_VEL,VECTOR,SAFETYZONE,LOGICBASED} oa_method;
oa_method OA_method_flag = PINGPONG; //0 =>No OA only opticflow 1=pingpong 2=>pot_heading 3=>pot_vel 4=>vector 5=>safetyzone
int8_t opti_speed_flag = 1;
float vref_max = 100;
/////////////////////////////////////////////////

//variables set by OA
float ref_pitch = 0.0;
float ref_roll = 0.0;

//initialize variables
float r_dot_new = 0.0;
float desired_vx = 0.0;
float desired_vy = 0.0;
float speed_pot = 0.0;
float yaw_diff = 0.0;
float alpha_fil = 1.0;
float heading_target = 0;
float new_heading = 0;
float v_desired = 0.0;

int32_t yaw_rate = 0;
float yaw_rate_write = 0;
float yaw_ref_write = 0;
int32_t keep_yaw_rate = 0;
int32_t keep_turning = 0;

float err_vx = 0;
float err_vy = 0;

float Total_Kan_x = 0;
float Total_Kan_y = 0;

struct NedCoor_f opti_speed_read;
struct FloatVect3 Total_force = {0, 0, 0};
//
//static void send_INPUT_CONTROL(void)
//{
//  //DOWNLINK_SEND_INPUT_CONTROL(DefaultChannel, DefaultDevice, &Total_Kan_x, &Total_Kan_y, &opticflow_stab.desired_vx,
//    //                          &opticflow_stab.desired_vy, &opti_speed_read.x, &opti_speed_read.y);
//}

void guidance_h_module_init(void)
{

}

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  /* Reset the integrated errors */
  opticflow_stab.err_vx_int = 0;
  opticflow_stab.err_vy_int = 0;

  /* Set rool/pitch to 0 degrees and psi to current heading */
  opticflow_stab.cmd.phi = 0;
  opticflow_stab.cmd.theta = 0;
  opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  new_heading = 0;

//  register_periodic_telemetry(DefaultPeriodic, "INPUT_CONTROL", send_INPUT_CONTROL);
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
void guidance_h_module_run(bool_t in_flight)
{
  //int vsupply_scaled=electrical.vsupply*10;

  OA_update();

  printf("phi: %i, theta: %i", opticflow_stab.cmd.phi, opticflow_stab.cmd.theta);
  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&opticflow_stab.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */
void OA_update()
{

  /* Check if we are in the correct AP_MODE before setting commands */
  //if (autopilot_mode != AP_MODE_MODULE){
  //  return;
  //}

  float v_x = 0;
  float v_y = 0;

  if (opti_speed_flag == 1) {
    //rotation_vector.psi = stateGetNedToBodyEulers_f()->psi;
    //opti_speed = stateGetSpeedNed_f();

    //Transform to body frame.
    //float_rmat_of_eulers_312(&T, &rotation_vector)Attractforce_goal_send;
    //MAT33_VECT3_MUL(*opti_speed, T, *opti_speed);

    // Calculate the speed in body frame
    struct FloatVect2 speed_cur;
    float psi = stateGetNedToBodyEulers_f()->psi;
    float s_psi = sin(psi);
    float c_psi = cos(psi);
    speed_cur.x = c_psi * stateGetSpeedNed_f()->x + s_psi * stateGetSpeedNed_f()->y;
    speed_cur.y = -s_psi * stateGetSpeedNed_f()->x + c_psi * stateGetSpeedNed_f()->y;

    opti_speed_read.x = speed_cur.x * 100;
    opti_speed_read.y = speed_cur.y * 100;
    //printf("%f, %f", opti_speed_read.x, opti_speed_read.y);

    //set result_vel
    v_x = speed_cur.y * 100;
    v_y = speed_cur.x * 100;
  } else {
  }

  if (OA_method_flag == NO_OBSTACLE_AVOIDANCE) {
    /* Calculate the error if we have enough flow */
    opticflow_stab.desired_vx = 0;
    opticflow_stab.desired_vy = 0;

    err_vx = opticflow_stab.desired_vx - v_x;
    err_vy = opticflow_stab.desired_vy - v_y;

    /* Calculate the integrated errors (TODO: bound??) */
    opticflow_stab.err_vx_int += err_vx / 100;
    opticflow_stab.err_vy_int += err_vy / 100;

    /* Calculate the commands */
    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vx / 100
                               + opticflow_stab.phi_igain * opticflow_stab.err_vx_int;
    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
                                 + opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

    /* Bound the roll and pitch commands */
    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
  }

  if (OA_method_flag == PINGPONG) {
    opticflow_stab.cmd.phi = ANGLE_BFP_OF_REAL(ref_roll);
    opticflow_stab.cmd.theta = ANGLE_BFP_OF_REAL(ref_pitch);
  }

  if (OA_method_flag == 2) {
    Total_Kan_x = r_dot_new;
    Total_Kan_y = speed_pot;

    alpha_fil = 0.1;

    yaw_rate = (int32_t)(alpha_fil * ANGLE_BFP_OF_REAL(r_dot_new));
    opticflow_stab.cmd.psi  = stateGetNedToBodyEulers_i()->psi + yaw_rate;

    INT32_ANGLE_NORMALIZE(opticflow_stab.cmd.psi);

    opticflow_stab.desired_vx = 0;
    opticflow_stab.desired_vy = speed_pot;

    /* Calculate the error if we have enough flow */

    err_vx = opticflow_stab.desired_vx - v_x;
    err_vy = opticflow_stab.desired_vy - v_y;

    /* Calculate the integrated errors (TODO: bound??) */
    opticflow_stab.err_vx_int += err_vx / 100;
    opticflow_stab.err_vy_int += err_vy / 100;

    /* Calculate the commands */
    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vx / 100
                               + opticflow_stab.phi_igain * opticflow_stab.err_vx_int;

    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
                                 + opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

    /* Bound the roll and pitch commands */
    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);

  }
  if (OA_method_flag == POT_HEADING) {
    new_heading = ref_pitch;

    opticflow_stab.desired_vx = sin(new_heading) * speed_pot * 100;
    opticflow_stab.desired_vy = cos(new_heading) * speed_pot * 100;

    /* Calculate the error if we have enough flow */
    err_vx = opticflow_stab.desired_vx - v_x;
    err_vy = opticflow_stab.desired_vy - v_y;


    /* Calculate the integrated errors (TODO: bound??) */
    opticflow_stab.err_vx_int += err_vx / 100;
    opticflow_stab.err_vy_int += err_vy / 100;

    /* Calculate the commands */
    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vx / 100
                               + opticflow_stab.phi_igain * opticflow_stab.err_vx_int;
    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
                                 + opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

    /* Bound the roll and pitch commands */
    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT)

  }

  if (OA_method_flag == VECTOR || OA_method_flag == SAFETYZONE || OA_method_flag == LOGICBASED) {
    //vector field method
    float v_desired_total;

    Total_Kan_x = ref_pitch;
    Total_Kan_y = ref_roll;

    //debug
    //float ref_diff = Attractforce_goal_send.x - ref_pitch;
    //printf("difference: %f %f %f \n", ref_pitch, Attractforce_goal_send.x,before);

    opticflow_stab.desired_vx = alpha_fil *
                                Total_Kan_y; //alpha_fil*(Repulsionforce_Kan.y+Attractforce_goal.y) + result->vel_x;
    opticflow_stab.desired_vy = alpha_fil *
                                Total_Kan_x; //alpha_fil*(Repulsionforce_Kan.x+Attractforce_goal.x) + result->vel_y;
    //printf("opticflow_stab.desired_vx: %f opticflow_stab.desired_vy: %f \n",opticflow_stab.desired_vx, opticflow_stab.desired_vy);

    v_desired_total = sqrt(opticflow_stab.desired_vx * opticflow_stab.desired_vx + opticflow_stab.desired_vy *
                           opticflow_stab.desired_vy);

    if (v_desired_total >= vref_max) {
      opticflow_stab.desired_vx = (vref_max / v_desired_total) * opticflow_stab.desired_vx;
      opticflow_stab.desired_vy = (vref_max / v_desired_total) * opticflow_stab.desired_vy;
    }

    /* Calculate the error if we have enough flow */

    //alpha_fil needs to be tuned!
    err_vx = opticflow_stab.desired_vx - v_x;
    err_vy = opticflow_stab.desired_vy - v_y;

    /* Calculate the integrated errors (TODO: bound??) */
    opticflow_stab.err_vx_int += err_vx / 100;
    opticflow_stab.err_vy_int += err_vy / 100;

    /* Calculate the commands */
    opticflow_stab.cmd.phi   = opticflow_stab.phi_pgain * err_vx / 100
                               + opticflow_stab.phi_igain * opticflow_stab.err_vx_int;
    opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
                                 + opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

    /* Bound the roll and pitch commands */
    BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
    BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);

  }
}
