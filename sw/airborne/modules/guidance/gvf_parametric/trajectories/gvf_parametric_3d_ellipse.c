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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_ellipse.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 3D ellipse (intersection between a cylinder and a tilted plane)
 */

#include "modules/nav/common_nav.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "gvf_parametric_3d_ellipse.h"

/*! Default gain kx for the 3d ellipse trajectory */
#ifndef GVF_PARAMETRIC_3D_ELLIPSE_KX
#define GVF_PARAMETRIC_3D_ELLIPSE_KX 0.001
#endif

/*! Default gain ky for the 3d ellipse trajectory */
#ifndef GVF_PARAMETRIC_3D_ELLIPSE_KY
#define GVF_PARAMETRIC_3D_ELLIPSE_KY 0.001
#endif

/*! Default gain kz for the 3d ellipse trajectory */
#ifndef GVF_PARAMETRIC_3D_ELLIPSE_KZ
#define GVF_PARAMETRIC_3D_ELLIPSE_KZ 0.001
#endif

/*! Default radius of the cylinder */
#ifndef GVF_PARAMETRIC_3D_ELLIPSE_R
#define GVF_PARAMETRIC_3D_ELLIPSE_R 80
#endif

/*! Default highest point for the ellipse trajectory */
#ifndef GVF_PARAMETRIC_3D_ELLIPSE_ZL
#define GVF_PARAMETRIC_3D_ELLIPSE_ZL 40
#endif

/*! Default highest point for the ellipse trajectory */
#ifndef GVF_PARAMETRIC_3D_ELLIPSE_ZH
#define GVF_PARAMETRIC_3D_ELLIPSE_ZH 40
#endif

/*! Default orientation in degrees for the lowest point of the ellipse */
#ifndef GVF_PARAMETRIC_3D_ELLIPSE_ALPHA
#define GVF_PARAMETRIC_3D_ELLIPSE_ALPHA 0
#endif

gvf_par_3d_ell_par gvf_parametric_3d_ellipse_par = {GVF_PARAMETRIC_3D_ELLIPSE_KX,
                                                  GVF_PARAMETRIC_3D_ELLIPSE_KY, GVF_PARAMETRIC_3D_ELLIPSE_KZ, GVF_PARAMETRIC_3D_ELLIPSE_R, GVF_PARAMETRIC_3D_ELLIPSE_ZL, GVF_PARAMETRIC_3D_ELLIPSE_ZH, GVF_PARAMETRIC_3D_ELLIPSE_ALPHA
                                                 };

void gvf_parametric_3d_ellipse_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
                                  float *f1dd, float *f2dd, float *f3dd)
{
  float xo = gvf_parametric_trajectory.p_parametric[0];
  float yo = gvf_parametric_trajectory.p_parametric[1];
  float r = gvf_parametric_trajectory.p_parametric[2];
  float zl = gvf_parametric_trajectory.p_parametric[3];
  float zh = gvf_parametric_trajectory.p_parametric[4];
  float alpha_rad = gvf_parametric_trajectory.p_parametric[5]*M_PI/180;

  float w = gvf_parametric_control.w;
  float wb = w * gvf_parametric_control.beta * gvf_parametric_control.s;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w'
  *f1 = r * cosf(wb) + xo;
  *f2 = r * sinf(wb) + yo;
  *f3 = 0.5 * (zh + zl + (zl - zh) * sinf(alpha_rad - wb));

  *f1d = -r * sinf(wb);
  *f2d = r * cosf(wb);
  *f3d = -0.5 * (zl - zh) * cosf(alpha_rad - wb);

  *f1dd = -r * cosf(wb);
  *f2dd = -r * sinf(wb);
  *f3dd = -0.5 * (zl - zh) * sinf(alpha_rad - wb);
}

