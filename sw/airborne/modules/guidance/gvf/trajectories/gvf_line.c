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

/** \file gvf_line.c
 *
 *  Guidance algorithm based on vector fields
 *  2D straight line trajectory
 */


#include "modules/nav/common_nav.h"
#include "gvf_line.h"
#include "generated/airframe.h"

/*! Gain ke for the line trajectory*/
#ifndef GVF_LINE_KE
#define GVF_LINE_KE 1
#endif

/*! Gain kn for the line trajectory*/
#ifndef GVF_LINE_KN
#define GVF_LINE_KN 1
#endif

/*! Default heading in degrees for a trajectory called from gvf_line_**_HEADING */
#ifndef GVF_LINE_HEADING
#define GVF_LINE_HEADING 0
#endif

/*! In case of tracking a segment, how much distance in meters will go the vehicle beyond the point x1,y1 before turning back */
#ifndef GVF_SEGMENT_D1
#define GVF_SEGMENT_D1 0
#endif

/*! In case of tracking a segment, how much distance in meters will go the vehicle beyond the point x2,y2 before turning back */
#ifndef GVF_SEGMENT_D2
#define GVF_SEGMENT_D2 0
#endif

gvf_li_par gvf_line_par = {GVF_LINE_KE, GVF_LINE_KN, GVF_LINE_HEADING};
gvf_seg_par gvf_segment_par = {GVF_SEGMENT_D1, GVF_SEGMENT_D2};

void gvf_line_info(float *phi, struct gvf_grad *grad,
                   struct gvf_Hess *hess)
{

  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;
  float a = gvf_trajectory.p[0];
  float b = gvf_trajectory.p[1];
  float alpha = gvf_trajectory.p[2];

  // Phi(x,y)
  *phi = -(px - a) * cosf(alpha) + (py - b) * sinf(alpha);

  // grad Phi
  grad->nx =  -cosf(alpha);
  grad->ny =   sinf(alpha);

  // Hessian Phi
  hess->H11 = 0;
  hess->H12 = 0;
  hess->H21 = 0;
  hess->H22 = 0;
}
