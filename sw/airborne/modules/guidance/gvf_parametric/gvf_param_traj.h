/*
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

#ifndef GVF_PARAMETRIC_TRAJ_H
#define GVF_PARAMETRIC_TRAJ_H

// Define only one segment by default
#ifndef GVF_PARAMETRIC_2D_BEZIER_N_SEG
#define GVF_PARAMETRIC_2D_BEZIER_N_SEG 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_common.h"
#include "std.h"

// Parameters for the trajectories
enum trajectories_parametric {
  TREFOIL_2D = 0,
  ELLIPSE_3D = 1,
  LISSAJOUS_3D = 2,
  BEZIER_2D = 3,
  NONE_PARAMETRIC = 255,
};

typedef struct {
  enum trajectories_parametric type;
  float p_parametric[16];
  int p_len;
} gvf_parametric_tra;

// Cubic bezier
typedef struct {
  float p0[2];
  float p1[2];
  float p2[2];
  float p3[2];
} bezier_t;

extern gvf_parametric_tra gvf_parametric_trajectory;

/** ------------------------------------------------------------------------ **/

extern void gvf_parametric_2d_trefoil_info(float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd);
extern void gvf_parametric_3d_ellipse_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
    float *f1dd, float *f2dd, float *f3dd);
extern void gvf_parametric_3d_lissajous_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
    float *f1dd, float *f2dd, float *f3dd);

// Bezier
extern void create_bezier_spline(bezier_t *bezier, float *px, float *py);
extern void gvf_parametric_2d_bezier_splines_info(bezier_t *bezier, float *f1, float *f2, float *f1d, float *f2d,
    float *f1dd, float *f2dd);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_TRAJ_H