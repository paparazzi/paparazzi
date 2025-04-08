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
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 2D trefoil knot
 */

#ifndef GVF_PARAMETRIC_2D_TREFOIL_H
#define GVF_PARAMETRIC_2D_TREFOIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_parametric/gvf_param_traj.h"

/** @typedef gvf_2d_tre_par
* @brief Parameters for the GVF parametric 2D trefoil knot
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
* @param w1 1st frequency
* @param w2 2nd frequency
* @param off Off-phase
* @param r Radius of the "circles"
* @param alpha Orientation/rotation of the trajectory in the XY plane
*/
typedef struct {
  float kx;
  float ky;
  float w1;
  float w2;
  float ratio;
  float r;
  float alpha;
} gvf_par_2d_tre_par;

extern gvf_par_2d_tre_par gvf_parametric_2d_trefoil_par;

// 2D Trefoil
extern bool nav_gvf_parametric_2D_trefoil_XY(float xo, float yo, float w1, float w2, float ratio, float r, float alpha);
extern bool nav_gvf_parametric_2D_trefoil_wp(uint8_t wp, float w1, float w2, float ratio, float r, float alpha);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_2D_TREFOIL_H
