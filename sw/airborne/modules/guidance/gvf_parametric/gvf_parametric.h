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

#define GVF_PARAMETRIC_GRAVITY 9.806

/*! Default gain kroll for tuning the "coordinated turn" */
#ifndef GVF_PARAMETRIC_CONTROL_KROLL
#define GVF_PARAMETRIC_CONTROL_KROLL 1
#endif

/*! Default gain kclimb for tuning the climbing setting point */
#ifndef GVF_PARAMETRIC_CONTROL_KCLIMB
#define GVF_PARAMETRIC_CONTROL_KCLIMB 1
#endif

/*! Default scale for the error signals */
#ifndef GVF_PARAMETRIC_CONTROL_L
#define GVF_PARAMETRIC_CONTROL_L 0.1
#endif

/*! Default scale for w  */
#ifndef GVF_PARAMETRIC_CONTROL_BETA
#define GVF_PARAMETRIC_CONTROL_BETA 0.01
#endif

/*! Default gain kpsi for tuning the alignment of the vehicle with the vector field */
#ifndef GVF_PARAMETRIC_CONTROL_KPSI
#define GVF_PARAMETRIC_CONTROL_KPSI 1
#endif

/*! Default GCS trajectory painter */
#ifndef GVF_OCAML_GCS
#define GVF_OCAML_GCS true
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_ellipse.h"
#include "modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_lissajous.h"
#include "modules/guidance/gvf_parametric/trajectories/gvf_parametric_2d_trefoil.h"

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
  float k_roll;
  float k_climb;
  float k_psi;
  float L;
  float beta;
} gvf_parametric_con;

extern gvf_parametric_con gvf_parametric_control;

// Parameters for the trajectories
enum trajectories_parametric {
  TREFOIL_2D = 0,
  ELLIPSE_3D = 1,
  LISSAJOUS_3D = 2,
  NONE_PARAMETRIC = 255,
};

typedef struct {
  enum trajectories_parametric type;
  float p_parametric[16];
  float phi_errors[3];
} gvf_parametric_tra;

extern gvf_parametric_tra gvf_parametric_trajectory;

// Init function
extern void gvf_parametric_init(void);

// Control functions
extern void gvf_parametric_set_direction(int8_t s);
extern void gvf_parametric_control_2D(float, float, float, float, float, float, float, float);
extern void gvf_parametric_control_3D(float, float, float, float, float, float, float, float, float,
                                      float, float, float);

// 2D Trefoil
extern bool gvf_parametric_2D_trefoil_XY(float, float, float, float, float, float, float);
extern bool gvf_parametric_2D_trefoil_wp(uint8_t, float, float, float, float, float);

// 3D Ellipse
extern bool gvf_parametric_3D_ellipse_XYZ(float, float, float, float, float, float);
extern bool gvf_parametric_3D_ellipse_wp(uint8_t, float, float, float, float);
extern bool gvf_parametric_3D_ellipse_wp_delta(uint8_t, float, float, float, float);

// 3D Lissajous
extern bool gvf_parametric_3D_lissajous_XYZ(float, float, float, float, float, float, float, float, float, float, float,
    float, float);
extern bool gvf_parametric_3D_lissajous_wp_center(uint8_t, float, float, float, float, float, float, float, float,
    float, float, float);

#ifdef __cplusplus
}
#endif


#endif // GVF_PARAMETRIC_H
