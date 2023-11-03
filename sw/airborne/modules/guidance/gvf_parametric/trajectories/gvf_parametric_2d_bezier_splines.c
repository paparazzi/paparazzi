/*
 * Copyright (C) 2023 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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

#include "modules/nav/common_nav.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "modules/guidance/gvf_parametric/trajectories/gvf_parametric_2d_bezier_splines.h"

#ifndef GVF_PARAMETRIC_2D_BEZIER_SPLINES_KX
#define GVF_PARAMETRIC_2D_BEZIER_SPLINES_KX 2.0
#endif

#ifndef GVF_PARAMETRIC_2D_BEZIER_SPLINES_KY
#define GVF_PARAMETRIC_2D_BEZIER_SPLINES_KY 2.0
#endif

gvf_par_2d_bezier_par gvf_parametric_2d_bezier_par = {GVF_PARAMETRIC_2D_BEZIER_SPLINES_KX,
                                                      GVF_PARAMETRIC_2D_BEZIER_SPLINES_KY
                                                     };

// Bezier is just an array
void create_bezier_spline(bezier_t *bezier, float *px, float *py)
{

  int k, j;
  j = 0;
  for (k = 0; k < GVF_PARAMETRIC_2D_BEZIER_N_SEG; k++) {
    bezier[k].p0[0] = px[j];
    bezier[k].p0[1] = py[j];
    bezier[k].p1[0] = px[j + 1];
    bezier[k].p1[1] = py[j + 1];
    bezier[k].p2[0] = px[j + 2];
    bezier[k].p2[1] = py[j + 2];
    bezier[k].p3[0] = px[j + 3];
    bezier[k].p3[1] = py[j + 3];

    // This allows for C^0 continuity (last point is init point)
    j += 3;
  }
}

void gvf_parametric_2d_bezier_splines_info(bezier_t *bezier, float *f1, float *f2, float *f1d, float *f2d, float *f1dd,
    float *f2dd)
{
  // How can we select in which bezier curve are we? Check w. spline zero: 0 <= t <= 1, spline ones: 1 <= t <= 2;
  float t = gvf_parametric_control.w;
  int n_seg = floorl(t);
  float tt = t - n_seg;
  if (n_seg < 0) {
    n_seg = 0;  // w could be < 0 in that case go to first point of first segment
    tt = 0;
  }
  // Evalute the corresponding bezier curve
  float p0x = bezier[n_seg].p0[0]; float p0y = bezier[n_seg].p0[1];
  float p1x = bezier[n_seg].p1[0]; float p1y = bezier[n_seg].p1[1];
  float p2x = bezier[n_seg].p2[0]; float p2y = bezier[n_seg].p2[1];
  float p3x = bezier[n_seg].p3[0]; float p3y = bezier[n_seg].p3[1];

  // Bézier curves

  // Curve (x,y)
  *f1 = (1 - tt) * (1 - tt) * (1 - tt) * p0x + 3 * (1 - tt) * (1 - tt) * tt * p1x + 3 *
        (1 - tt) * tt * tt * p2x + tt * tt * tt * p3x;
  *f2 = (1 - tt) * (1 - tt) * (1 - tt) * p0y + 3 * (1 - tt) * (1 - tt) * tt * p1y + 3 *
        (1 - tt) * tt * tt * p2y + tt * tt * tt * p3y;

  // First derivative
  *f1d = 3 * (1 - tt) * (1 - tt) * (p1x - p0x) + 6 * (1 - tt) * tt * (p2x - p1x) + 3 * tt * tt * (p3x - p2x);
  *f2d = 3 * (1 - tt) * (1 - tt) * (p1y - p0y) + 6 * (1 - tt) * tt * (p2y - p1y) + 3 * tt * tt * (p3y - p2y);

  // Second derivative
  *f1dd = 6 * (1 - tt) * (p2x - 2 * p1x + p0x) + 6 * tt * (p3x - 2 * p2x + p1x);
  *f2dd = 6 * (1 - tt) * (p2y - 2 * p1y + p0y) + 6 * tt * (p3y - 2 * p2y + p1y);


}
