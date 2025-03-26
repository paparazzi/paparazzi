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

#include "nav_ellipse.h"

#include "generated/airframe.h"
#include "modules/guidance/gvf_ik/gvf_ik.h"

/*! Default gain ke for the ellipse trajectory */
#ifndef GVF_IK_ELLIPSE_KE
#define GVF_IK_ELLIPSE_KE 1
#endif

/*! Default gain kn for the ellipse trajectory */
#ifndef GVF_IK_ELLIPSE_KN
#define GVF_IK_ELLIPSE_KN 1
#endif

/*! Default first axis for the ellipse trajectory */
#ifndef GVF_IK_ELLIPSE_A
#define GVF_IK_ELLIPSE_A 80
#endif

/*! Default second axis for the ellipse trajectory */
#ifndef GVF_IK_ELLIPSE_B
#define GVF_IK_ELLIPSE_B 80
#endif

/*! Default orientation in degrees for the ellipse trajectory */
#ifndef GVF_IK_ELLIPSE_ALPHA
#define GVF_IK_ELLIPSE_ALPHA 0
#endif

gvf_ik_ell_par gvf_ik_ellipse_par = {
  GVF_IK_ELLIPSE_KE, GVF_IK_ELLIPSE_KN, GVF_IK_ELLIPSE_A, GVF_IK_ELLIPSE_B, GVF_IK_ELLIPSE_ALPHA
};

// Param array lenght
static int gvf_p_len_wps = 0;

/** ------------------------------------------------------------------------ **/

// ELLIPSE 

bool gvf_ik_ellipse_XY(float x, float y, float a, float b, float alpha)
{
  float e;
  struct gvf_grad grad_ellipse;
  struct gvf_Hess Hess_ellipse;

  gvf_trajectory.type = 1;
  gvf_trajectory.p[0] = x;
  gvf_trajectory.p[1] = y;
  gvf_trajectory.p[2] = a;
  gvf_trajectory.p[3] = b;
  gvf_trajectory.p[4] = alpha;
  gvf_trajectory.p_len= 5 + gvf_p_len_wps;
  gvf_p_len_wps = 0;

  // SAFE MODE
  if (a < 1 || b < 1) {
    gvf_trajectory.p[2] = 60;
    gvf_trajectory.p[3] = 60;
  }

  if ((int)gvf_trajectory.p[2] == (int)gvf_trajectory.p[3]) {
    gvf_setNavMode(GVF_MODE_CIRCLE);

  } else {
    gvf_setNavMode(GVF_MODE_WAYPOINT);
  }

  gvf_ellipse_info(&e, &grad_ellipse, &Hess_ellipse);
  gvf_ik_control.ke = gvf_ik_ellipse_par.ke;
  gvf_ik_control_2D(gvf_ik_ellipse_par.ke, gvf_ik_ellipse_par.kn,
                 e, &grad_ellipse, &Hess_ellipse);

  gvf_ik_control.error = e;

  return true;
}


bool gvf_ik_ellipse_wp(uint8_t wp, float a, float b, float alpha)
{  
  gvf_trajectory.p[5] = wp;
  gvf_p_len_wps = 1;

  gvf_ik_ellipse_XY(WaypointX(wp),  WaypointY(wp), a, b, alpha);
  return true;
}
