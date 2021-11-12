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

/** \file gvf_sin.c
 *
 *  Guidance algorithm based on vector fields
 *  2D sinusoidal trajectory
 */


#include "modules/nav/common_nav.h"
#include "gvf_sin.h"
#include "generated/airframe.h"

/*! Default gain ke for the sin trajectory*/
#ifndef GVF_SIN_KE
#define GVF_SIN_KE 1
#endif

/*! Default gain kn for the sin trajectory*/
#ifndef GVF_SIN_KN
#define GVF_SIN_KN 1
#endif

/*! Default orientation in rads for the sin trajectory function gvf_sin_**_alpha*/
#ifndef GVF_SIN_ALPHA
#define GVF_SIN_ALPHA 0
#endif

/*! Default frequency for the sin trajectory in rads*/
#ifndef GVF_SIN_W
#define GVF_SIN_W 0
#endif

/*! Default off-set in rads for the sin trajectory in rads*/
#ifndef GVF_SIN_OFF
#define GVF_SIN_OFF 0
#endif

/*! Default amplitude for the sin trajectory in meters*/
#ifndef GVF_SIN_A
#define GVF_SIN_A 0
#endif

gvf_s_par gvf_sin_par = {GVF_SIN_KE, GVF_SIN_KN,
                         GVF_SIN_ALPHA, GVF_SIN_W, GVF_SIN_OFF, GVF_SIN_A
                        };


void gvf_sin_info(float *phi, struct gvf_grad *grad,
                  struct gvf_Hess *hess)
{

  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;
  float a = gvf_trajectory.p[0];
  float b = gvf_trajectory.p[1];
  float alpha = gvf_trajectory.p[2];
  float w = gvf_trajectory.p[3];
  float off = gvf_trajectory.p[4];
  float A = gvf_trajectory.p[5];

  float cosa = cosf(alpha);
  float sina = sinf(alpha);

  // Phi(x,y)
  float xs = (px - a) * sina - (py - b) * cosa;
  float ys =  -(px - a) * cosa - (py - b) * sina;

  // TODO Make it always in (-pi, pi] in an efficient way
  float ang = (w * xs + off);
  float cosang = cosf(ang);
  float sinang = sinf(ang);

  *phi = ys - A * sinang;

  // grad Phi
  grad->nx =  -cosa - A * w * sina * cosang;
  grad->ny =  -sina + A * w * cosa * cosang;

  // Hessian Phi
  hess->H11 =  A * w * w * sina * sina * sinang;
  hess->H12 = -A * w * w * sina * cosa * sinang;
  hess->H21 = -A * w * w * cosa * sina * sinang;
  hess->H22 =  A * w * w * cosa * cosa * sinang;
}
