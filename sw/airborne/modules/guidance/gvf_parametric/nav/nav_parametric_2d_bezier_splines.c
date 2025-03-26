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

#include "nav_parametric_2d_bezier_splines.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"

#ifndef GVF_PARAMETRIC_2D_BEZIER_SPLINES_KX
#define GVF_PARAMETRIC_2D_BEZIER_SPLINES_KX 2.0
#endif

#ifndef GVF_PARAMETRIC_2D_BEZIER_SPLINES_KY
#define GVF_PARAMETRIC_2D_BEZIER_SPLINES_KY 2.0
#endif

gvf_par_2d_bezier_par gvf_parametric_2d_bezier_par = {
  GVF_PARAMETRIC_2D_BEZIER_SPLINES_KX, GVF_PARAMETRIC_2D_BEZIER_SPLINES_KY
};

bezier_t gvf_bezier_2D[GVF_PARAMETRIC_2D_BEZIER_N_SEG];

/** ------------------------------------------------------------------------ **/

// 2D CUBIC BEZIER CURVE

bool gvf_parametric_2D_bezier_XY(void)
{
  gvf_parametric_trajectory.type = BEZIER_2D;
  float fx, fy, fxd, fyd, fxdd, fydd;
  gvf_parametric_2d_bezier_splines_info(gvf_bezier_2D, &fx, &fy, &fxd, &fyd, &fxdd, &fydd);
  gvf_parametric_control_2D(gvf_parametric_2d_bezier_par.kx, gvf_parametric_2d_bezier_par.ky, fx, fy, fxd, fyd, fxdd,
                            fydd);
  return true;
}

/* @param first_wp is the first waypoint of the Bézier Spline
 * there should be 3*GVF_PARAMETRIC_2D_BEZIER_N_SEG+1 points
 */
bool gvf_parametric_2D_bezier_wp(uint8_t first_wp)
{
  float x[3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1];
  float y[3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1];
  int k;
  for (k = 0; k < 3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1; k++) {
    x[k] = WaypointX(first_wp + k);
    y[k] = WaypointY(first_wp + k);
  }
  create_bezier_spline(gvf_bezier_2D, x, y);

  /* Send data piecewise. Some radio modules do not allow for a big data frame.*/

  // Send x points -> Indicate x with sign (+) in the first parameter
  if (gvf_parametric_telemetry.splines_ctr == 0) {
    gvf_parametric_trajectory.p_parametric[0] = -GVF_PARAMETRIC_2D_BEZIER_N_SEG; // send x (negative value)
    for (k = 0; k < 3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1; k++) {
      gvf_parametric_trajectory.p_parametric[k + 1] = x[k];
    }
  }
  // Send y points -> Indicate y with sign (-) in the first parameter
  else if (gvf_parametric_telemetry.splines_ctr == 1) {
    gvf_parametric_trajectory.p_parametric[0] = GVF_PARAMETRIC_2D_BEZIER_N_SEG; // send y (positive value)
    for (k = 0; k < 3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1; k++) {
      gvf_parametric_trajectory.p_parametric[k + 1] = y[k];
    }
  }
  // send kx, ky, beta and anything else needed..
  else {
    gvf_parametric_trajectory.p_parametric[0] = 0.0;
    gvf_parametric_trajectory.p_parametric[1] = gvf_parametric_2d_bezier_par.kx;
    gvf_parametric_trajectory.p_parametric[2] = gvf_parametric_2d_bezier_par.ky;
    gvf_parametric_trajectory.p_parametric[3] = gvf_parametric_control.beta;
  }

  gvf_parametric_trajectory.p_len = 16;

  // restart the spline
  if (gvf_parametric_control.w >= (float)GVF_PARAMETRIC_2D_BEZIER_N_SEG) {
    gvf_parametric_control.w = 0;
  } else if (gvf_parametric_control.w < 0) {
    gvf_parametric_control.w = 0;
  }
  gvf_parametric_2D_bezier_XY();
  return true;
}

