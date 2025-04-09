/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/guidance/gvf_parametric/gvf_parametric.h
 *
 * Guiding vector field algorithm for 2D and 3D parametric trajectories.
 */

#ifndef GVF_PARAMETRIC_H
#define GVF_PARAMETRIC_H

/*! Default GCS trajectory painter */
#ifndef GVF_OCAML_GCS
#define GVF_OCAML_GCS true
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_common.h"
#include "modules/guidance/trajectories/gvf_param_traj.h"

/** @typedef gvf_parametric_con
* @brief Control parameters for the GVF_PARAMETRIC
* @param w Virtual coordinate from the parametrization of the trajectory
* @param delta_T Time between iterations needed for integrating w
* @param s Defines the direction to be tracked. It takes the values -1 or 1.
* @param k_roll Gain for tuning the coordinated turn.
* @param k_climb Gain for tuning the climbing setting point.
*/
typedef struct {
  float w;
  float delta_T;
  int8_t s;
  float k_psi;
  float L;
  float beta;
} gvf_parametric_con;

typedef struct {
  float phi_errors[3];
  int e_len;
  uint32_t splines_ctr;
} gvf_parametric_tel;

extern gvf_parametric_con gvf_parametric_control;
extern gvf_parametric_tel gvf_parametric_telemetry;

// Init function
extern void gvf_parametric_init(void);

// Control functions
extern void gvf_parametric_set_direction(int8_t s);
extern void gvf_parametric_control_2D(float, float, float, float, float, float, float, float);
extern void gvf_parametric_control_3D(float, float, float, float, float, float, float, float, float,
                                      float, float, float);

#ifdef __cplusplus
}
#endif


#endif // GVF_PARAMETRIC_H
