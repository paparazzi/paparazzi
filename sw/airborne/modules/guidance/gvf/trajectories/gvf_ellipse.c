/*
 * Copyright (C) 2016  Hector Garcia de Marina
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file gvf_ellipse.c
 *
 *  Guidance algorithm based on vector fields
 *  2D Ellipse trajectory
 */


#include "modules/nav/common_nav.h"
#include "gvf_ellipse.h"
#include "generated/airframe.h"

/*! Default gain ke for the ellipse trajectory */
#ifndef GVF_ELLIPSE_KE
#define GVF_ELLIPSE_KE 1
#endif

/*! Default gain kn for the ellipse trajectory */
#ifndef GVF_ELLIPSE_KN
#define GVF_ELLIPSE_KN 1
#endif

/*! Default first axis for the ellipse trajectory */
#ifndef GVF_ELLIPSE_A
#define GVF_ELLIPSE_A 80
#endif

/*! Default second axis for the ellipse trajectory */
#ifndef GVF_ELLIPSE_B
#define GVF_ELLIPSE_B 80
#endif

/*! Default orientation in degrees for the ellipse trajectory */
#ifndef GVF_ELLIPSE_ALPHA
#define GVF_ELLIPSE_ALPHA 0
#endif

gvf_ell_par gvf_ellipse_par = {GVF_ELLIPSE_KE, GVF_ELLIPSE_KN,
                               GVF_ELLIPSE_A, GVF_ELLIPSE_B, GVF_ELLIPSE_ALPHA
                              };

void gvf_ellipse_info(float *phi, struct gvf_grad *grad,
                      struct gvf_Hess *hess)
{

  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;
  float wx = gvf_trajectory.p[0];
  float wy = gvf_trajectory.p[1];
  float a = gvf_trajectory.p[2];
  float b = gvf_trajectory.p[3];
  float alpha = gvf_trajectory.p[4];

  float cosa = cosf(alpha);
  float sina = sinf(alpha);

  // Phi(x,y)
  float xel = (px - wx) * cosa - (py - wy) * sina;
  float yel = (px - wx) * sina + (py - wy) * cosa;
  *phi = (xel / a) * (xel / a) + (yel / b) * (yel / b) - 1;

  // grad Phi
  grad->nx = (2 * xel / (a * a)) * cosa + (2 * yel / (b * b)) * sina;
  grad->ny = (2 * yel / (b * b)) * cosa - (2 * xel / (a * a)) * sina;

  // Hessian Phi
  hess->H11 = 2 * (cosa * cosa / (a * a)
                   + sina * sina / (b * b));
  hess->H12 = 2 * sina * cosa * (1 / (b * b) - 1 / (a * a));
  hess->H21 = hess->H12;
  hess->H22 = 2 * (sina * sina / (a * a)
                   + cosa * cosa / (b * b));
}
