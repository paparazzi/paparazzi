/*
 * Copyright (C) 2015 Roland + Clint
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/obstacle_avoidance/guidance_OA.h
 *  @brief Guidance for the obstacle avoidance methods
 */
#ifndef GUIDANCE_OA
#define GUIDANCE_OA

#include "std.h"
#include "modules/computer_vision/lib/v4l/v4l2.h"
//#include "inter_thread_data.h"
#include "math/pprz_algebra_int.h"

/* The opticflow stabilization */
struct opticflow_stab_t {
  int32_t phi_pgain;        ///< The roll P gain on the err_vx
  int32_t phi_igain;        ///< The roll I gain on the err_vx_int
  int32_t theta_pgain;      ///< The pitch P gain on the err_vy
  int32_t theta_igain;      ///< The pitch I gain on the err_vy_int
  float desired_vx;         ///< The desired velocity in the x direction (cm/s)
  float desired_vy;         ///< The desired velocity in the y direction (cm/s)

  float err_vx_int;         ///< The integrated velocity error in x direction (m/s)
  float err_vy_int;         ///< The integrated velocity error in y direction (m/s)
  struct Int32Eulers cmd;   ///< The commands that are send to the hover loop
};
extern struct opticflow_stab_t opticflow_stab;

extern int8_t filter_flag;
extern int8_t repulsionforce_filter_flag;

typedef enum {NO_OBSTACLE_AVOIDANCE, PINGPONG, POT_HEADING, POT_VEL, VECTOR, SAFETYZONE, LOGICBASED} oa_method;
extern oa_method OA_method_flag; //0 =>No OA only opticflow 1=pingpong 2=>pot_heading 3=>pot_vel 4=>vector 5=>safetyzone

extern int8_t opti_speed_flag;
extern float vref_max;

//variables form optic flow module
extern float ref_pitch;
extern float ref_roll;
extern float r_dot_new;
extern float speed_pot;

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

// Update the stabiliztion commands based on a vision result
extern void OA_update(void);

#endif
