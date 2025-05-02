/*
 * Copyright (C) 2016  Hector Garcia de Marina
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

/** @file gvf.h
 *
 *  Guidance algorithm based on vector fields
 */

#ifndef GVF_IK_H
#define GVF_IK_H

#define GVF_GRAVITY 9.806

/*! GVF-IK settings */
#ifndef GVF_IK_GAMMA_AMPLITUDE
#define GVF_IK_GAMMA_AMPLITUDE 0
#endif

#ifndef GVF_IK_GAMMA_OMEGA
#define GVF_IK_GAMMA_OMEGA 0
#endif

/*! Default GCS trajectory painter */
#ifndef GVF_OCAML_GCS
#define GVF_OCAML_GCS true
#endif

#include "std.h"
#include "modules/guidance/gvf_common.h"
#include "modules/guidance/trajectories/gvf_traj.h"

/** STRUCTS ---------------------------------------------------------------- **/

/** @typedef gvf_con
* @brief Control parameters for the GVF
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehile to the vector field
* @param error Error signal. It does not have any specific units. It depends on how the trajectory has been implemented. Check the specific wiki entry for each trajectory.
* @param s Defines the direction to be tracked. Its meaning depends on the trajectory and its implementation. Check the wiki entry of the GVF. It takes the values -1 or 1.
*/
typedef struct {
  float ke;
  float kn;
  float phi;
  float error;
  float error_n;
  int8_t s;

  float gamma_amplitude;
  float gamma_omega;
  float gamma;
  float gamma_dot;
} gvf_ik_con;

typedef struct {
  float n_norm;
  float t_norm;
  float omega_d;
  float omega;
} gvf_ik_tel;

// Extern structs
extern gvf_ik_con gvf_ik_control;

/** EXTERN FUNCTIONS ------------------------------------------------------- **/

extern void gvf_ik_init(void);
extern void gvf_ik_control_2D(float ke, float kn, float e, struct gvf_grad *, struct gvf_Hess *);
extern void gvf_ik_set_direction(int8_t s);

#endif // GVF_IK_H
