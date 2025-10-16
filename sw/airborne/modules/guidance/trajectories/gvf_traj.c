/*
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

#include "gvf_traj.h"

// Trajectory
gvf_tra gvf_trajectory = {NONE, {0}, 1};
gvf_seg gvf_segment;

/** ------------------------------------------------------------------------ **/

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
