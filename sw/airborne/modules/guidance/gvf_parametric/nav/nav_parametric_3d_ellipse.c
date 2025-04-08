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

#include "nav_parametric_3d_ellipse.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"

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

gvf_par_3d_ell_par gvf_parametric_3d_ellipse_par = {
  GVF_PARAMETRIC_3D_ELLIPSE_KX, GVF_PARAMETRIC_3D_ELLIPSE_KY, GVF_PARAMETRIC_3D_ELLIPSE_KZ, 
  GVF_PARAMETRIC_3D_ELLIPSE_R, GVF_PARAMETRIC_3D_ELLIPSE_ZL, GVF_PARAMETRIC_3D_ELLIPSE_ZH, 
  GVF_PARAMETRIC_3D_ELLIPSE_ALPHA
};

#ifdef FIXEDWING_FIRMWARE

static int gvf_parametric_p_len_wps = 0;

/** ------------------------------------------------------------------------ **/

// 3D ELLIPSE

bool nav_gvf_parametric_3D_ellipse_XYZ(float xo, float yo, float r, float zl, float zh, float alpha)
{
  horizontal_mode = HORIZONTAL_MODE_CIRCLE; //  Circle for the 2D GCS

  // Safety first! If the asked altitude is low
  if (zl > zh) {
    zl = zh;
  }
  if (zl < 1 || zh < 1) {
    zl = 10;
    zh = 10;
  }
  if (r < 1) {
    r = 60;
  }

  gvf_parametric_trajectory.type = ELLIPSE_3D;
  gvf_parametric_trajectory.p_parametric[0] = xo;
  gvf_parametric_trajectory.p_parametric[1] = yo;
  gvf_parametric_trajectory.p_parametric[2] = r;
  gvf_parametric_trajectory.p_parametric[3] = zl;
  gvf_parametric_trajectory.p_parametric[4] = zh;
  gvf_parametric_trajectory.p_parametric[5] = alpha;
  gvf_parametric_trajectory.p_len = 6 + gvf_parametric_p_len_wps;
  gvf_parametric_p_len_wps = 0;

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;

  gvf_parametric_3d_ellipse_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
  gvf_parametric_control_3D(gvf_parametric_3d_ellipse_par.kx, gvf_parametric_3d_ellipse_par.ky,
                            gvf_parametric_3d_ellipse_par.kz, f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

  return true;
}

bool nav_gvf_parametric_3D_ellipse_wp(uint8_t wp, float r, float zl, float zh, float alpha)
{
  gvf_parametric_trajectory.p_parametric[6] = wp;
  gvf_parametric_p_len_wps = 1;

  nav_gvf_parametric_3D_ellipse_XYZ(waypoints[wp].x, waypoints[wp].y, r, zl, zh, alpha);
  return true;
}

bool nav_gvf_parametric_3D_ellipse_wp_delta(uint8_t wp, float r, float alt_center, float delta, float alpha)
{
  float zl = alt_center - delta;
  float zh = alt_center + delta;

  nav_gvf_parametric_3D_ellipse_XYZ(waypoints[wp].x, waypoints[wp].y, r, zl, zh, alpha);
  return true;
}

#endif