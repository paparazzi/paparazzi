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
 * 3D lissajous
 */

#include "nav_parametric_3d_lissajous.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"

/*! Default gain kx for the 3d lissajous trajectory */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_KX
#define GVF_PARAMETRIC_3D_LISSAJOUS_KX 0.001
#endif

/*! Default gain ky for the 3d lissajous trajectory */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_KY
#define GVF_PARAMETRIC_3D_LISSAJOUS_KY 0.001
#endif

/*! Default gain kz for the 3d lissajous trajectory */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_KZ
#define GVF_PARAMETRIC_3D_LISSAJOUS_KZ 0.001
#endif

/*! Default amplitude of the trajectory in the X plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_CX
#define GVF_PARAMETRIC_3D_LISSAJOUS_CX 80
#endif

/*! Default amplitude of the trajectory in the Y plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_CY
#define GVF_PARAMETRIC_3D_LISSAJOUS_CY 80
#endif

/*! Default amplitude of the trajectory in the Z plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_CZ
#define GVF_PARAMETRIC_3D_LISSAJOUS_CZ 10
#endif

/*! Default frequency of the trajectory in the X plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_WX
#define GVF_PARAMETRIC_3D_LISSAJOUS_WX 1
#endif

/*! Default frequency of the trajectory in the Y plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_WY
#define GVF_PARAMETRIC_3D_LISSAJOUS_WY 1
#endif

/*! Default frequency of the trajectory in the Z plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_WZ
#define GVF_PARAMETRIC_3D_LISSAJOUS_WZ 1
#endif

/*! Default offphase of the trajectory in the X plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_DX
#define GVF_PARAMETRIC_3D_LISSAJOUS_DX 0
#endif

/*! Default offphase of the trajectory in the Y plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_DY
#define GVF_PARAMETRIC_3D_LISSAJOUS_DY 0
#endif

/*! Default offphase of the trajectory in the Z plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_DZ
#define GVF_PARAMETRIC_3D_LISSAJOUS_DZ 0
#endif

/*! Default orientation in degrees for the trajectory in the XY plane */
#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_ALPHA
#define GVF_PARAMETRIC_3D_LISSAJOUS_ALPHA 0
#endif

gvf_par_3d_lis_par gvf_parametric_3d_lissajous_par = {
  GVF_PARAMETRIC_3D_LISSAJOUS_KX, GVF_PARAMETRIC_3D_LISSAJOUS_KY, GVF_PARAMETRIC_3D_LISSAJOUS_KZ, 
  GVF_PARAMETRIC_3D_LISSAJOUS_CX, GVF_PARAMETRIC_3D_LISSAJOUS_CY, GVF_PARAMETRIC_3D_LISSAJOUS_CZ, 
  GVF_PARAMETRIC_3D_LISSAJOUS_WX, GVF_PARAMETRIC_3D_LISSAJOUS_WY, GVF_PARAMETRIC_3D_LISSAJOUS_WZ, 
  GVF_PARAMETRIC_3D_LISSAJOUS_DX, GVF_PARAMETRIC_3D_LISSAJOUS_DY, GVF_PARAMETRIC_3D_LISSAJOUS_DZ, 
  GVF_PARAMETRIC_3D_LISSAJOUS_ALPHA
};

#ifdef FIXEDWING_FIRMWARE

static int gvf_parametric_p_len_wps = 0;

/** ------------------------------------------------------------------------ **/

// 3D Lissajous

bool nav_gvf_parametric_3D_lissajous_XYZ(float xo, float yo, float zo, float cx, float cy, float cz, float wx, float wy,
                                     float wz, float dx, float dy, float dz, float alpha)
{
  // Safety first! If the asked altitude is low
  if ((zo - cz) < 1) {
    zo = 10;
    cz = 0;
  }

  gvf_parametric_trajectory.type = LISSAJOUS_3D;
  gvf_parametric_trajectory.p_parametric[0] = xo;
  gvf_parametric_trajectory.p_parametric[1] = yo;
  gvf_parametric_trajectory.p_parametric[2] = zo;
  gvf_parametric_trajectory.p_parametric[3] = cx;
  gvf_parametric_trajectory.p_parametric[4] = cy;
  gvf_parametric_trajectory.p_parametric[5] = cz;
  gvf_parametric_trajectory.p_parametric[6] = wx;
  gvf_parametric_trajectory.p_parametric[7] = wy;
  gvf_parametric_trajectory.p_parametric[8] = wz;
  gvf_parametric_trajectory.p_parametric[9] = dx;
  gvf_parametric_trajectory.p_parametric[10] = dy;
  gvf_parametric_trajectory.p_parametric[11] = dz;
  gvf_parametric_trajectory.p_parametric[12] = alpha;
  gvf_parametric_trajectory.p_len = 13 + gvf_parametric_p_len_wps;
  gvf_parametric_p_len_wps = 0;

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;
  float wb = gvf_parametric_control.w * gvf_parametric_control.beta * gvf_parametric_control.s;

  gvf_parametric_3d_lissajous_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd, wb);
  gvf_parametric_control_3D(gvf_parametric_3d_lissajous_par.kx, gvf_parametric_3d_lissajous_par.ky,
                            gvf_parametric_3d_lissajous_par.kz, f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

  return true;
}

bool nav_gvf_parametric_3D_lissajous_wp_center(uint8_t wp, float zo, float cx, float cy, float cz, float wx, float wy,
    float wz, float dx, float dy, float dz, float alpha)
{
  gvf_parametric_trajectory.p_parametric[13] = wp;
  gvf_parametric_p_len_wps = 1;

  nav_gvf_parametric_3D_lissajous_XYZ(waypoints[wp].x, waypoints[wp].y, zo, cx, cy, cz, wx, wy, wz, dx, dy, dz, alpha);
  return true;
}

#endif