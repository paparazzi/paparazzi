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

#include "nav_line.h"

#include "generated/airframe.h"
#include "modules/guidance/gvf/gvf.h"

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

// Param array lenght
static int gvf_p_len_wps = 0;

/** ------------------------------------------------------------------------ **/

static void gvf_line(float a, float b, float heading)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 0;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = heading;
  gvf_trajectory.p_len= 3 + gvf_p_len_wps;
  gvf_p_len_wps = 0;

  gvf_line_info(&e, &grad_line, &Hess_line);
  gvf_control.ke = gvf_line_par.ke;
  gvf_control_2D(1e-2 * gvf_line_par.ke, gvf_line_par.kn, e, &grad_line, &Hess_line);

  gvf_control.error = e;
  
  gvf_setNavMode(GVF_MODE_WAYPOINT);
  
  gvf_segment.seg = 0;
}

static int out_of_segment_area(float x1, float y1, float x2, float y2, float d1, float d2)
{
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atan2f(zy, zx);

  float cosa = cosf(-alpha);
  float sina = sinf(-alpha);

  float pxr = px * cosa - py * sina;
  float zxr = zx * cosa - zy * sina;

  int s = 0;

  if (pxr < -d1) {
    s = 1;
  } else if (pxr > (zxr + d2)) {
    s = -1;
  }

  if (zy < 0) {
    s *= -1;
  }

  return s;
}

/** ------------------------------------------------------------------------ **/

// STRAIGHT LINE

bool gvf_line_XY_heading(float a, float b, float heading)
{
  gvf_set_direction(1);
  gvf_line(a, b, heading);
  return true;
}

bool gvf_line_XY1_XY2(float x1, float y1, float x2, float y2)
{ 
  if (gvf_p_len_wps != 2) {
    gvf_trajectory.p[3] = x2;
    gvf_trajectory.p[4] = y2;
    gvf_trajectory.p[5] = 0;
    gvf_p_len_wps = 3;
  }

  float zx = x2 - x1;
  float zy = y2 - y1;

  gvf_line_XY_heading(x1, y1, atan2f(zx, zy));
  
  gvf_setNavMode(GVF_MODE_ROUTE);
  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  gvf_p_len_wps = 2;
  
  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_segment_loop_XY1_XY2(float x1, float y1, float x2, float y2, float d1, float d2)
{
  int s = out_of_segment_area(x1, y1, x2, y2, d1, d2);
  if (s != 0) {
    gvf_set_direction(s);
  }

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atanf(zx / zy);

  gvf_line(x1, y1, alpha);

  gvf_setNavMode(GVF_MODE_ROUTE);
  
  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_segment_loop_wp1_wp2(uint8_t wp1, uint8_t wp2, float d1, float d2)
{ 
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  gvf_trajectory.p[5] = d1;
  gvf_trajectory.p[6] = d2;
  gvf_p_len_wps = 4;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_segment_loop_XY1_XY2(x1, y1, x2, y2, d1, d2);
}

bool gvf_segment_XY1_XY2(float x1, float y1, float x2, float y2)
{ 
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;

  float beta = atan2f(zy, zx);
  float cosb = cosf(-beta);
  float sinb = sinf(-beta);
  float zxr = zx * cosb - zy * sinb;
  float pxr = px * cosb - py * sinb;

  if ((zxr > 0 && pxr > zxr) || (zxr < 0 && pxr < zxr)) {
    return false;
  }

  return gvf_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_segment_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  gvf_p_len_wps = 2;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_segment_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_line_wp_heading(uint8_t wp, float heading)
{
  gvf_trajectory.p[3] = wp;
  gvf_p_len_wps = 1;

  heading = RadOfDeg(heading);

  float a = WaypointX(wp);
  float b = WaypointY(wp);

  return gvf_line_XY_heading(a, b, heading);
}