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
 * 2D trefoil know
 */

#include "nav_parametric_2d_trefoil.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"

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

gvf_par_2d_tre_par gvf_parametric_2d_trefoil_par = {
  GVF_PARAMETRIC_2D_TREFOIL_KX, GVF_PARAMETRIC_2D_TREFOIL_KY, 
  GVF_PARAMETRIC_2D_TREFOIL_W1, GVF_PARAMETRIC_2D_TREFOIL_W2, 
  GVF_PARAMETRIC_2D_TREFOIL_RATIO, GVF_PARAMETRIC_2D_TREFOIL_R, 
  GVF_PARAMETRIC_2D_TREFOIL_ALPHA
};

static int gvf_parametric_p_len_wps = 0;

/** ------------------------------------------------------------------------ **/

// 2D TREFOIL KNOT

bool gvf_parametric_2D_trefoil_XY(float xo, float yo, float w1, float w2, float ratio, float r, float alpha)
{
  gvf_parametric_trajectory.type = TREFOIL_2D;
  gvf_parametric_trajectory.p_parametric[0] = xo;
  gvf_parametric_trajectory.p_parametric[1] = yo;
  gvf_parametric_trajectory.p_parametric[2] = w1;
  gvf_parametric_trajectory.p_parametric[3] = w2;
  gvf_parametric_trajectory.p_parametric[4] = ratio;
  gvf_parametric_trajectory.p_parametric[5] = r;
  gvf_parametric_trajectory.p_parametric[6] = alpha;
  gvf_parametric_trajectory.p_len = 7 + gvf_parametric_p_len_wps;
  gvf_parametric_p_len_wps = 0;

  float f1, f2, f1d, f2d, f1dd, f2dd;

  gvf_parametric_2d_trefoil_info(&f1, &f2, &f1d, &f2d, &f1dd, &f2dd);
  gvf_parametric_control_2D(gvf_parametric_2d_trefoil_par.kx, gvf_parametric_2d_trefoil_par.ky, f1, f2, f1d, f2d, f1dd,
                            f2dd);

  return true;
}

bool gvf_parametric_2D_trefoil_wp(uint8_t wp, float w1, float w2, float ratio, float r, float alpha)
{
  gvf_parametric_trajectory.p_parametric[7] = wp;
  gvf_parametric_p_len_wps = 1;
  gvf_parametric_2D_trefoil_XY(WaypointX(wp), WaypointY(wp), w1, w2, ratio, r, alpha);
  return true;
}

