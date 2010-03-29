/*
 * $Id: nav_cube.c 3600 2009-07-01 20:05:12Z hecto $
 *  
 * Copyright (C) 2010  Martin Mueller
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

/** \file nav_cube.c
 *  \brief Navigation in a cube towards a center
 *
 */

#include "airframe.h"
#include "nav_cube.h"
#include "nav.h"

int32_t cube_alpha;
int32_t cube_size_x, cube_size_y, cube_size_z;
int32_t cube_grid_x, cube_grid_z;
int32_t cube_offs_x, cube_offs_y, cube_offs_z;
int32_t cube_nline_x, cube_nline_z;

bool_t nav_cube_init(uint8_t center, uint8_t tb, uint8_t te) {

  int32_t i, start_bx, start_by, start_bz, start_ex, start_ey, start_ez;
  int32_t bx, by, ex, ey;
  float alpha, cos_alpha, sin_alpha;

  /* sanity checks */
  if (cube_grid_x == 0) cube_nline_x = 1;
  else cube_nline_x = cube_size_x / cube_grid_x + 1;
  if (cube_grid_z == 0) cube_nline_z = 1;
  else cube_nline_z = cube_size_z / cube_grid_z + 1;

  /* do not do more than pre-set number of lines */
  if (cube_nline_x >= MAX_LINES_X) cube_nline_x = 0;

  /* do the costly stuff only once */
  alpha = ((360. - cube_alpha) / 360.) * 2 * M_PI;
  cos_alpha = cos(alpha);
  sin_alpha = sin(alpha);

  /* calculate lower left start begin/end x coord */
  start_bx = waypoints[center].x - (((cube_nline_x-1) * cube_grid_x)/2) + cube_offs_x;
  start_ex = waypoints[center].x - (((cube_nline_x-1) * cube_grid_x)/2) + cube_offs_x;

  /* calculate lower left start begin point y coord */
  start_by = waypoints[center].y - cube_offs_y - cube_size_y;

  /* calculate lower left start end point y coord */
  start_ey = waypoints[center].y - cube_offs_y;

  /* calculate lower left start begin/end z coord */
  start_bz = waypoints[center].a - (((cube_nline_z-1) * cube_grid_z)/2) + cube_offs_z;
  start_ez = waypoints[center].a - (((cube_nline_z-1) * cube_grid_z)/2) + cube_offs_z;

  for (i=0; i < cube_nline_x; i++) {
    /* set waypoints and vectorize in regard to center */
    bx = (start_bx + i*cube_grid_x) - waypoints[center].x;
    by = start_by - waypoints[center].y;
    ex = (start_ex + i*cube_grid_x) - waypoints[center].x;
    ey = start_ey - waypoints[center].y;
    /* rotate clockwise with alpha and un-vectorize*/
    waypoints[tb+i].x = bx * cos_alpha - by * sin_alpha + waypoints[center].x;
    waypoints[tb+i].y = bx * sin_alpha + by * cos_alpha + waypoints[center].y;
    waypoints[tb+i].a = start_bz;
    waypoints[te+i].x = ex * cos_alpha - ey * sin_alpha + waypoints[center].x;
    waypoints[te+i].y = ex * sin_alpha + ey * cos_alpha + waypoints[center].y;
    waypoints[te+i].a = start_ez;
  }

  /* bug in <for from="" to=""> ? */
  cube_nline_x--;
  cube_nline_z--;

  return FALSE; 
}

bool_t nav_cube(int8_t j, int8_t i,
                uint8_t dest_b, uint8_t dest_e, 
                uint8_t src_b, uint8_t src_e) {

  if (i > cube_nline_x) return FALSE;
  if (j > cube_nline_z) return FALSE;

  waypoints[dest_b].x = waypoints[src_b+i].x;
  waypoints[dest_b].y = waypoints[src_b+i].y;
  waypoints[dest_b].a = waypoints[src_b+i].a + j*cube_grid_z;

  waypoints[dest_e].x = waypoints[src_e+i].x;
  waypoints[dest_e].y = waypoints[src_e+i].y;
  waypoints[dest_e].a = waypoints[src_e+i].a + j*cube_grid_z;

  return FALSE; 
}

