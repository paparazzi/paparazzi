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

#include "nav_sin.h"

#include "generated/airframe.h"
#include "modules/guidance/gvf_ik/gvf_ik.h"

/*! Default gain ke for the sin trajectory*/
#ifndef GVF_IK_SIN_KE
#define GVF_IK_SIN_KE 1
#endif

/*! Default gain kn for the sin trajectory*/
#ifndef GVF_IK_SIN_KN
#define GVF_IK_SIN_KN 1
#endif

/*! Default orientation in rads for the sin trajectory function gvf_sin_**_alpha*/
#ifndef GVF_IK_SIN_ALPHA
#define GVF_IK_SIN_ALPHA 0
#endif

/*! Default frequency for the sin trajectory in rads*/
#ifndef GVF_IK_SIN_W
#define GVF_IK_SIN_W 0
#endif

/*! Default off-set in rads for the sin trajectory in rads*/
#ifndef GVF_IK_SIN_OFF
#define GVF_IK_SIN_OFF 0
#endif

/*! Default amplitude for the sin trajectory in meters*/
#ifndef GVF_IK_SIN_A
#define GVF_IK_SIN_A 0
#endif

gvf_ik_s_par gvf_ik_sin_par = {
  GVF_IK_SIN_KE, GVF_IK_SIN_KN, GVF_IK_SIN_ALPHA, GVF_IK_SIN_W, GVF_IK_SIN_OFF, GVF_IK_SIN_A
};

// Param array lenght
static int gvf_p_len_wps = 0;

/** ------------------------------------------------------------------------ **/

// SINUSOIDAL (if w = 0 and off = 0, then we just have the straight line case)

bool gvf_ik_sin_XY_alpha(float a, float b, float alpha, float w, float off, float A)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 2;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = alpha;
  gvf_trajectory.p[3] = w;
  gvf_trajectory.p[4] = off;
  gvf_trajectory.p[5] = A;
  gvf_trajectory.p_len= 6 + gvf_p_len_wps;
  gvf_p_len_wps = 0;

  gvf_sin_info(&e, &grad_line, &Hess_line);
  gvf_ik_control.ke = gvf_ik_sin_par.ke;
  gvf_ik_control_2D(1e-2 * gvf_ik_sin_par.ke, gvf_ik_sin_par.kn, e, &grad_line, &Hess_line);

  gvf_ik_control.error = e;

  return true;
}

bool gvf_ik_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off, float A)
{
  w = 2 * M_PI * w;

  gvf_trajectory.p[6] = wp1;
  gvf_trajectory.p[7] = wp2;
  gvf_p_len_wps = 2;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  float zx = x1 - x2;
  float zy = y1 - y2;

  float alpha = atanf(zy / zx);

  gvf_ik_sin_XY_alpha(x1, y1, alpha, w, off, A);

  return true;
}

bool gvf_ik_sin_wp_alpha(uint8_t wp, float alpha, float w, float off, float A)
{
  w = 2 * M_PI * w;
  alpha = RadOfDeg(alpha);

  gvf_trajectory.p[6] = wp;
  gvf_p_len_wps = 1;

  float x = WaypointX(wp);
  float y = WaypointY(wp);

  gvf_ik_sin_XY_alpha(x, y, alpha, w, off, A);

  return true;
}
