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
 * @file modules/guidance/gvf_advanced/gvf_advanced.h
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 */

#ifndef GVF_ADVANCED_H
#define GVF_ADVANCED_H

#define GVF_ADVANCED_GRAVITY 9.806

/*! Default gain kroll for tuning the "coordinated turn" */
#ifndef GVF_ADVANCED_CONTROL_KROLL
#define GVF_ADVANCED_CONTROL_KROLL 1
#endif

/*! Default gain kclimb for tuning the climbing setting point */
#ifndef GVF_ADVANCED_CONTROL_KCLIMB
#define GVF_ADVANCED_CONTROL_KCLIMB 1
#endif

/*! Default scale for the error signals */
#ifndef GVF_ADVANCED_CONTROL_L
#define GVF_ADVANCED_CONTROL_L 0.1
#endif

/*! Default scale for w  */
#ifndef GVF_ADVANCED_CONTROL_BETA
#define GVF_ADVANCED_CONTROL_BETA 0.01
#endif

/*! Default gain kpsi for tuning the alignment of the vehicle with the vector field */
#ifndef GVF_ADVANCED_CONTROL_KPSI
#define GVF_ADVANCED_CONTROL_KPSI 1
#endif


#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_advanced/trajectories/gvf_advanced_3d_ellipse.h"

/** @typedef gvf_advanced_con
* @brief Control parameters for the GVF_ADVANCED
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
} gvf_advanced_con;

extern gvf_advanced_con gvf_advanced_control;

// Parameters for the trajectories
enum trajectories_advanced {
  ELLIPSE_3D = 0,
  NONE_ADVANCED = 255,
};

typedef struct {
  enum trajectories_advanced type;
  float p_advanced[16];
} gvf_advanced_tra;

extern gvf_advanced_tra gvf_advanced_trajectory;

// Init function
extern void gvf_advanced_init(void);

// Control functions
extern void gvf_advanced_control_3D(float, float, float, float, float, float, float, float, float,
                                    float, float, float);

// 3D Ellipse
extern bool gvf_advanced_3D_ellipse_XY(float, float, float, float, float, float);
extern bool gvf_advanced_3D_ellipse_wp(uint8_t, float, float, float, float);
extern bool gvf_advanced_3D_ellipse_wp_delta(uint8_t, float, float, float, float);

#ifdef __cplusplus
}
#endif


#endif // GVF_ADVANCED_H
