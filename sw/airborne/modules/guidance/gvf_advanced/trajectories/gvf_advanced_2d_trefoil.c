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
 * @file modules/guidance/gvf_advanced/trajectories/gvf_advanced_2d_trefoil.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 2D trefoil know
 */

#include "subsystems/navigation/common_nav.h"
#include "modules/guidance/gvf_advanced/gvf_advanced.h"
#include "gvf_advanced_2d_trefoil.h"

/*! Default gain kx for the 2d trefoil knot trajectory */
#ifndef GVF_ADVANCED_2D_TREFOIL_KX
#define GVF_ADVANCED_2D_TREFOIL_KX 0.001
#endif

/*! Default gain ky for the 2d trefoil knot trajectory */
#ifndef GVF_ADVANCED_2D_TREFOIL_KY
#define GVF_ADVANCED_2D_TREFOIL_KY 0.001
#endif

/*! Default 1st frequency for the 2d trefoil trajectory*/
#ifndef GVF_ADVANCED_2D_TREFOIL_W1
#define GVF_ADVANCED_2D_TREFOIL_W1 0.02
#endif

/*! Default 2nd frequency for the 2d trefoil trajectory*/
#ifndef GVF_ADVANCED_2D_TREFOIL_W2
#define GVF_ADVANCED_2D_TREFOIL_W2 0.03
#endif

/*! Default ratio for the 2d trefoil trajectory*/
#ifndef GVF_ADVANCED_2D_TREFOIL_RATIO
#define GVF_ADVANCED_2D_TREFOIL_RATIO 160
#endif

/*! Default radius of the circles for the 2d trefoil trajectory*/
#ifndef GVF_ADVANCED_2D_TREFOIL_R
#define GVF_ADVANCED_2D_TREFOIL_R 80
#endif

gvf_adv_2d_tre_par gvf_advanced_2d_trefoil_par = {GVF_ADVANCED_2D_TREFOIL_KX, GVF_ADVANCED_2D_TREFOIL_KY, GVF_ADVANCED_2D_TREFOIL_W1, GVF_ADVANCED_2D_TREFOIL_W2, GVF_ADVANCED_2D_TREFOIL_RATIO, GVF_ADVANCED_2D_TREFOIL_R};

void gvf_advanced_2d_trefoil_info(float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd)
{
  float xo = gvf_advanced_trajectory.p_advanced[0];
  float yo = gvf_advanced_trajectory.p_advanced[1];
  float w1 = gvf_advanced_trajectory.p_advanced[2];
  float w2 = gvf_advanced_trajectory.p_advanced[3];
  float ratio = gvf_advanced_trajectory.p_advanced[4];
  float r = gvf_advanced_trajectory.p_advanced[5];

  float w = gvf_advanced_control.w;
  float wb = w * gvf_advanced_control.beta;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w'
  *f1 = cosf(wb*w1)*(r*cosf(wb*w2) + ratio) + xo;
  *f2 = sinf(wb*w1)*(r*cosf(wb*w2) + ratio) + yo;

  *f1d = -w1*sinf(wb*w1)*(r*cosf(wb*w2) + ratio) - cosf(wb*w1)*r*w2*sinf(wb*w2);
  *f2d =  w1*cosf(wb*w1)*(r*cosf(wb*w2) + ratio) - sinf(wb*w1)*r*w2*sinf(wb*w2);

  *f1dd = -w1*w1*cosf(wb*w1)*(r*cosf(wb*w2) + ratio) + w1*sinf(wb*w1)*r*w2*sinf(wb*w2) + w1*sinf(wb*w1)*r*w2*sinf(wb*w2) - cosf(wb*w1)*r*w2*w2*cosf(wb*w2);
  *f2dd = -w1*w1*sinf(wb*w1)*(r*cosf(wb*w2) + ratio) - w1*cosf(wb*w1)*r*w2*sinf(wb*w2) - w1*cosf(wb*w1)*r*w2*sinf(wb*w2) - sinf(wb*w1)*r*w2*w2*cosf(wb*w2);
}

