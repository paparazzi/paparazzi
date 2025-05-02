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

#include "gvf_param_traj.h"

// Trajectory
gvf_parametric_tra gvf_parametric_trajectory = {NONE_PARAMETRIC, {0}, 1};

/** ------------------------------------------------------------------------ **/

void gvf_parametric_2d_trefoil_info(
  float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd, 
  float wb)
{
  float xo = gvf_parametric_trajectory.p_parametric[0];
  float yo = gvf_parametric_trajectory.p_parametric[1];
  float w1 = gvf_parametric_trajectory.p_parametric[2];
  float w2 = gvf_parametric_trajectory.p_parametric[3];
  float ratio = gvf_parametric_trajectory.p_parametric[4];
  float r = gvf_parametric_trajectory.p_parametric[5];
  float alpha_rad = gvf_parametric_trajectory.p_parametric[6]*M_PI/180;

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

void gvf_parametric_3d_ellipse_info(
  float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d, float *f1dd, float *f2dd, float *f3dd, 
  float wb)
{
  float xo = gvf_parametric_trajectory.p_parametric[0];
  float yo = gvf_parametric_trajectory.p_parametric[1];
  float r = gvf_parametric_trajectory.p_parametric[2];
  float zl = gvf_parametric_trajectory.p_parametric[3];
  float zh = gvf_parametric_trajectory.p_parametric[4];
  float alpha_rad = gvf_parametric_trajectory.p_parametric[5]*M_PI/180;

  // Parametric equations of the trajectory and the partial derivatives w.r.t. 'w'
  *f1 = r * cosf(wb) + xo;
  *f2 = r * sinf(wb) + yo;
  *f3 = 0.5 * (zh + zl + (zl - zh) * sinf(alpha_rad - wb));

  *f1d = -r * sinf(wb);
  *f2d = r * cosf(wb);
  *f3d = -0.5 * (zl - zh) * cosf(alpha_rad - wb);

  *f1dd = -r * cosf(wb);
  *f2dd = -r * sinf(wb);
  *f3dd = -0.5 * (zl - zh) * sinf(alpha_rad - wb);
}

void gvf_parametric_3d_lissajous_info(
  float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d, float *f1dd, float *f2dd, float *f3dd,
  float wb)
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

// BEZIER

void gvf_parametric_2d_bezier_splines_info(
  bezier_t *bezier, float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd,
  float w)
{
  // How can we select in which bezier curve are we? Check w. spline zero: 0 <= t <= 1, spline ones: 1 <= t <= 2;
  float t = w;
  int n_seg = floorl(t);
  float tt = t - n_seg;
  if (n_seg < 0) {
    n_seg = 0;  // w could be < 0 in that case go to first point of first segment
    tt = 0;
  }
  // Evalute the corresponding bezier curve
  float p0x = bezier[n_seg].p0[0]; float p0y = bezier[n_seg].p0[1];
  float p1x = bezier[n_seg].p1[0]; float p1y = bezier[n_seg].p1[1];
  float p2x = bezier[n_seg].p2[0]; float p2y = bezier[n_seg].p2[1];
  float p3x = bezier[n_seg].p3[0]; float p3y = bezier[n_seg].p3[1];

  // Bézier curves

  // Curve (x,y)
  *f1 = (1 - tt) * (1 - tt) * (1 - tt) * p0x + 3 * (1 - tt) * (1 - tt) * tt * p1x + 3 *
        (1 - tt) * tt * tt * p2x + tt * tt * tt * p3x;
  *f2 = (1 - tt) * (1 - tt) * (1 - tt) * p0y + 3 * (1 - tt) * (1 - tt) * tt * p1y + 3 *
        (1 - tt) * tt * tt * p2y + tt * tt * tt * p3y;

  // First derivative
  *f1d = 3 * (1 - tt) * (1 - tt) * (p1x - p0x) + 6 * (1 - tt) * tt * (p2x - p1x) + 3 * tt * tt * (p3x - p2x);
  *f2d = 3 * (1 - tt) * (1 - tt) * (p1y - p0y) + 6 * (1 - tt) * tt * (p2y - p1y) + 3 * tt * tt * (p3y - p2y);

  // Second derivative
  *f1dd = 6 * (1 - tt) * (p2x - 2 * p1x + p0x) + 6 * tt * (p3x - 2 * p2x + p1x);
  *f2dd = 6 * (1 - tt) * (p2y - 2 * p1y + p0y) + 6 * tt * (p3y - 2 * p2y + p1y);
}