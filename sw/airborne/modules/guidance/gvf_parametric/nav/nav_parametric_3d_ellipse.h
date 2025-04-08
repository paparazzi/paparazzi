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
 * 3D ellipse (intersection between a cylinder and a tilted plane)
 */

#ifndef GVF_PARAMETRIC_3D_ELLIPSE_H
#define GVF_PARAMETRIC_3D_ELLIPSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_parametric/gvf_param_traj.h"

/** @typedef gvf_3d_ell_par
* @brief Parameters for the GVF parametric 3D ellipse
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
* @param kz Gain defining how agressive is the vector field in z coordinate
* @param r Radius of the cylinder in meters
* @param zl Altitude of the lowest point of the ellipse
* @param zh Altitude of the highest point of the ellipse
* @param alpha Heading of the lowest point zl in rads
*/
typedef struct {
  float kx;
  float ky;
  float kz;
  float r;
  float zl;
  float zh;
  float alpha;
} gvf_par_3d_ell_par;

extern gvf_par_3d_ell_par gvf_parametric_3d_ellipse_par;

// 3D Ellipse
extern bool nav_gvf_parametric_3D_ellipse_XYZ(float xo, float yo, float r, float zl, float zh, float alpha);
extern bool nav_gvf_parametric_3D_ellipse_wp(uint8_t wp, float r, float zl, float zh, float alpha);
extern bool nav_gvf_parametric_3D_ellipse_wp_delta(uint8_t wp, float r, float alt_center, float delta, float alpha);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_3D_ELLIPSE_H
