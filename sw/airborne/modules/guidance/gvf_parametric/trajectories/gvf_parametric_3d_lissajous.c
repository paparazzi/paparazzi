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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_lissajous.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 3D lissajous
 */

#include "modules/nav/common_nav.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "gvf_parametric_3d_lissajous.h"

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

gvf_par_3d_lis_par gvf_parametric_3d_lissajous_par = {GVF_PARAMETRIC_3D_LISSAJOUS_KX, GVF_PARAMETRIC_3D_LISSAJOUS_KY, GVF_PARAMETRIC_3D_LISSAJOUS_KZ, GVF_PARAMETRIC_3D_LISSAJOUS_CX, GVF_PARAMETRIC_3D_LISSAJOUS_CY, GVF_PARAMETRIC_3D_LISSAJOUS_CZ, GVF_PARAMETRIC_3D_LISSAJOUS_WX, GVF_PARAMETRIC_3D_LISSAJOUS_WY, GVF_PARAMETRIC_3D_LISSAJOUS_WZ, GVF_PARAMETRIC_3D_LISSAJOUS_DX, GVF_PARAMETRIC_3D_LISSAJOUS_DY, GVF_PARAMETRIC_3D_LISSAJOUS_DZ, GVF_PARAMETRIC_3D_LISSAJOUS_ALPHA};

void gvf_parametric_3d_lissajous_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
                                  float *f1dd, float *f2dd, float *f3dd)
{
  float xo = gvf_parametric_trajectory.p_parametric[0];
  float yo = gvf_parametric_trajectory.p_parametric[1];
  float zo = gvf_parametric_trajectory.p_parametric[2];
  float cx = gvf_parametric_trajectory.p_parametric[3];
  float cy = gvf_parametric_trajectory.p_parametric[4];
  float cz = gvf_parametric_trajectory.p_parametric[5];
  float wx = gvf_parametric_trajectory.p_parametric[6];
  float wy = gvf_parametric_trajectory.p_parametric[7];
  float wz = gvf_parametric_trajectory.p_parametric[8];
  float deltax_rad = gvf_parametric_trajectory.p_parametric[9]*M_PI/180;
  float deltay_rad = gvf_parametric_trajectory.p_parametric[10]*M_PI/180;
  float deltaz_rad = gvf_parametric_trajectory.p_parametric[11]*M_PI/180;
  float alpha_rad = gvf_parametric_trajectory.p_parametric[12]*M_PI/180;

  float w = gvf_parametric_control.w;
  float wb = w * gvf_parametric_control.beta * gvf_parametric_control.s;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w'

  float nrf1 = cx*cosf(wx*wb + deltax_rad);
  float nrf2 = cy*cosf(wy*wb + deltay_rad);

  *f1 = cosf(alpha_rad)*nrf1 - sinf(alpha_rad)*nrf2 + xo;
  *f2 = sinf(alpha_rad)*nrf1 + cosf(alpha_rad)*nrf2 + yo;
  *f3 = cz*cosf(wz*wb + deltaz_rad) + zo;

  float nrf1d = -wx*cx*sinf(wx*wb + deltax_rad);
  float nrf2d = -wy*cy*sinf(wy*wb + deltay_rad);

  *f1d = cosf(alpha_rad)*nrf1d - sinf(alpha_rad)*nrf2d;
  *f2d = sinf(alpha_rad)*nrf1d + cosf(alpha_rad)*nrf2d;
  *f3d = -wz*cz*sinf(wz*wb + deltaz_rad);

  float nrf1dd = -wx*wx*cx*cosf(wx*wb + deltax_rad);
  float nrf2dd = -wy*wy*cy*cosf(wy*wb + deltay_rad);

  *f1dd = cosf(alpha_rad)*nrf1dd - sinf(alpha_rad)*nrf2dd;
  *f2dd = sinf(alpha_rad)*nrf1dd + cosf(alpha_rad)*nrf2dd;
  *f3dd = -wz*wz*cz*cosf(wz*wb + deltaz_rad);
}

