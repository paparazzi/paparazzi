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

#ifndef GVF_IK_ELLIPSE_H
#define GVF_IK_ELLIPSE_H

#include "modules/guidance/trajectories/gvf_traj.h"

/** @typedef gvf_ell_par
* @brief Parameters for the GVF line trajectory
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehile to the vector field
* @param a First axis of the ellipse in meters
* @param b Second axis of the ellipse in meters
* @param alpha Orientation of the ellipse in rads
*/
typedef struct {
  float ke;
  float kn;
  float a;
  float b;
  float alpha;
} gvf_ik_ell_par;

/** ------------------------------------------------------------------------ **/

extern gvf_ik_ell_par gvf_ik_ellipse_par;

// Ellipse
extern bool nav_gvf_ik_ellipse_wp(uint8_t wp, float a, float b, float alpha);
extern bool nav_gvf_ik_ellipse_XY(float x, float y, float a, float b, float alpha);

#endif // GVF_IK_ELLIPSE_H
