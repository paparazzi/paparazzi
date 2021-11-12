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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_2d_trefoil.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 2D trefoil know
 */

#include "modules/nav/common_nav.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "gvf_parametric_2d_trefoil.h"

/*! Default gain kx for the 2d trefoil knot trajectory */
#ifndef GVF_PARAMETRIC_2D_TREFOIL_KX
#define GVF_PARAMETRIC_2D_TREFOIL_KX 0.001
#endif

/*! Default gain ky for the 2d trefoil knot trajectory */
#ifndef GVF_PARAMETRIC_2D_TREFOIL_KY
#define GVF_PARAMETRIC_2D_TREFOIL_KY 0.001
#endif

/*! Default 1st frequency for the 2d trefoil trajectory*/
#ifndef GVF_PARAMETRIC_2D_TREFOIL_W1
#define GVF_PARAMETRIC_2D_TREFOIL_W1 0.02
#endif

/*! Default 2nd frequency for the 2d trefoil trajectory*/
#ifndef GVF_PARAMETRIC_2D_TREFOIL_W2
#define GVF_PARAMETRIC_2D_TREFOIL_W2 0.03
#endif

/*! Default ratio for the 2d trefoil trajectory*/
#ifndef GVF_PARAMETRIC_2D_TREFOIL_RATIO
#define GVF_PARAMETRIC_2D_TREFOIL_RATIO 160
#endif

/*! Default radius of the circles for the 2d trefoil trajectory*/
#ifndef GVF_PARAMETRIC_2D_TREFOIL_R
#define GVF_PARAMETRIC_2D_TREFOIL_R 80
#endif

/*! Default orientation for the 2d trefoil trajectory*/
#ifndef GVF_PARAMETRIC_2D_TREFOIL_ALPHA
#define GVF_PARAMETRIC_2D_TREFOIL_ALPHA 0
#endif

gvf_par_2d_tre_par gvf_parametric_2d_trefoil_par = {GVF_PARAMETRIC_2D_TREFOIL_KX, GVF_PARAMETRIC_2D_TREFOIL_KY, GVF_PARAMETRIC_2D_TREFOIL_W1, GVF_PARAMETRIC_2D_TREFOIL_W2, GVF_PARAMETRIC_2D_TREFOIL_RATIO, GVF_PARAMETRIC_2D_TREFOIL_R, GVF_PARAMETRIC_2D_TREFOIL_ALPHA};

void gvf_parametric_2d_trefoil_info(float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd)
{
  float xo = gvf_parametric_trajectory.p_parametric[0];
  float yo = gvf_parametric_trajectory.p_parametric[1];
  float w1 = gvf_parametric_trajectory.p_parametric[2];
  float w2 = gvf_parametric_trajectory.p_parametric[3];
  float ratio = gvf_parametric_trajectory.p_parametric[4];
  float r = gvf_parametric_trajectory.p_parametric[5];
  float alpha_rad = gvf_parametric_trajectory.p_parametric[6]*M_PI/180;

  float w = gvf_parametric_control.w;
  float wb = w * gvf_parametric_control.beta * gvf_parametric_control.s;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w'
  float nrf1 = cosf(wb*w1)*(r*cosf(wb*w2) + ratio);
  float nrf2 = sinf(wb*w1)*(r*cosf(wb*w2) + ratio);

  float nrf1d = -w1*sinf(wb*w1)*(r*cosf(wb*w2) + ratio) - cosf(wb*w1)*r*w2*sinf(wb*w2);
  float nrf2d =  w1*cosf(wb*w1)*(r*cosf(wb*w2) + ratio) - sinf(wb*w1)*r*w2*sinf(wb*w2);

  float nrf1dd = -w1*w1*cosf(wb*w1)*(r*cosf(wb*w2) + ratio) + w1*sinf(wb*w1)*r*w2*sinf(wb*w2) + w1*sinf(wb*w1)*r*w2*sinf(wb*w2) - cosf(wb*w1)*r*w2*w2*cosf(wb*w2);
  float nrf2dd = -w1*w1*sinf(wb*w1)*(r*cosf(wb*w2) + ratio) - w1*cosf(wb*w1)*r*w2*sinf(wb*w2) - w1*cosf(wb*w1)*r*w2*sinf(wb*w2) - sinf(wb*w1)*r*w2*w2*cosf(wb*w2);

  *f1 = cosf(alpha_rad)*nrf1 - sinf(alpha_rad)*nrf2 + xo;
  *f2 = sinf(alpha_rad)*nrf1 + cosf(alpha_rad)*nrf2 + yo;

  *f1d = cosf(alpha_rad)*nrf1d - sinf(alpha_rad)*nrf2d;
  *f2d = sinf(alpha_rad)*nrf1d + cosf(alpha_rad)*nrf2d;

  *f1dd = cosf(alpha_rad)*nrf1dd - sinf(alpha_rad)*nrf2dd;
  *f2dd = sinf(alpha_rad)*nrf1dd + cosf(alpha_rad)*nrf2dd;
}

