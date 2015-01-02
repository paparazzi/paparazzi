/*
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
 */

/**
 * @file modules/nav/nav_cube.c
 *
 * Fixedwing Navigation in a cube towards a center.
 *
 */

#include "generated/airframe.h"
#include "modules/nav/nav_cube.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"

#define MAX_LINES_X 8
#define STBY_OFFSET 500

struct NavCube nav_cube;

bool_t nav_cube_setup(uint8_t center, uint8_t tb, uint8_t te)
{

  int32_t j, start_bx, start_by, start_bz, start_ex, start_ey, start_ez;
  int32_t bx, by, ex, ey;
  float alpha, cos_alpha, sin_alpha;
  int32_t cube_nline_x_t, cube_nline_z_t;
  int32_t cube_pos_x, cube_pos_z;
  int32_t cube_line_x_start, cube_line_x_end;
  int32_t cube_line_z_start, cube_line_z_end;

  /* sanity checks */
  if (nav_cube.nsect_x <= 0) {
    nav_cube.nsect_x = 1;
  }
  if (nav_cube.nsect_z <= 0) {
    nav_cube.nsect_z = 1;
  }
  if ((nav_cube.sect <= 0) || (nav_cube.sect > (nav_cube.nsect_x * nav_cube.nsect_z))) {
    nav_cube.sect = 1;
  }

  /* total number of lines/layers to fly */
  if (nav_cube.grid_x == 0) {
    cube_nline_x_t = 1;
  } else {
    cube_nline_x_t = nav_cube.size.x / nav_cube.grid_x + 1;
  }
  if (nav_cube.grid_z == 0) {
    cube_nline_z_t = 1;
  } else {
    cube_nline_z_t = nav_cube.size.z / nav_cube.grid_z + 1;
  }

  /* position and number of lines in this sector */
  cube_pos_x = (nav_cube.sect - 1) % nav_cube.nsect_x;
  cube_line_x_start = (cube_pos_x    * cube_nline_x_t) / nav_cube.nsect_x;
  cube_line_x_end   = ((cube_pos_x + 1) * cube_nline_x_t) / nav_cube.nsect_x;
  if (cube_line_x_end > cube_nline_x_t) {
    cube_line_x_end = cube_nline_x_t;
  }
  nav_cube.nline_x = cube_line_x_end - cube_line_x_start;

  /* do not do more than pre-set number of lines */
  if (nav_cube.nline_x >= MAX_LINES_X) { nav_cube.nline_x = MAX_LINES_X - 1; }

  /* position and number of layers in this sector */
  cube_pos_z = (nav_cube.sect - 1) / nav_cube.nsect_x;
  cube_line_z_start = (cube_pos_z    * cube_nline_z_t) / nav_cube.nsect_z;
  cube_line_z_end   = ((cube_pos_z + 1) * cube_nline_z_t) / nav_cube.nsect_z;
  nav_cube.nline_z = cube_line_z_end - cube_line_z_start;

  /* do the costly stuff only once */
  alpha = ((360. - nav_cube.alpha) / 360.) * 2 * M_PI;
  cos_alpha = cos(alpha);
  sin_alpha = sin(alpha);

  /* calculate lower left start begin/end x coord */
  start_bx = WaypointX(center) - (((cube_nline_x_t - 1) * nav_cube.grid_x) / 2)
             + nav_cube.offset.x;
  start_ex = start_bx;

  /* calculate lower left start end point y coord */
  start_ey = WaypointY(center) - nav_cube.offset.y;

  /* calculate lower left start begin point y coord */
  start_by = start_ey - nav_cube.size.y;

  /* calculate lower left start begin/end z coord */
  start_bz = waypoints[center].a - (((cube_nline_z_t - 1) * nav_cube.grid_z) / 2)
             + (cube_line_z_start * nav_cube.grid_z) + nav_cube.offset.z;
  start_ez = start_bz;

  /* reset all waypoints to the standby position */
  for (j = 0; j < MAX_LINES_X; j++) {
    waypoints[tb + j].x = WaypointX(center) + STBY_OFFSET;
    waypoints[tb + j].y = WaypointY(center);
    waypoints[te + j].x = WaypointX(center) + STBY_OFFSET;
    waypoints[te + j].y = WaypointY(center);
  }

  /* set used waypoints */
  for (j = 0; j < nav_cube.nline_x; j++) {
    int i = cube_line_x_start + j;
    /* set waypoints and vectorize in regard to center */
    bx = (start_bx + i * nav_cube.grid_x) - WaypointX(center);
    by = start_by - WaypointY(center);
    ex = (start_ex + i * nav_cube.grid_x) - WaypointX(center);
    ey = start_ey - WaypointY(center);
    /* rotate clockwise with alpha and un-vectorize*/
    waypoints[tb + j].x = bx * cos_alpha - by * sin_alpha + WaypointX(center);
    waypoints[tb + j].y = bx * sin_alpha + by * cos_alpha + WaypointY(center);
    waypoints[tb + j].a = start_bz;
    waypoints[te + j].x = ex * cos_alpha - ey * sin_alpha + WaypointX(center);
    waypoints[te + j].y = ex * sin_alpha + ey * cos_alpha + WaypointY(center);
    waypoints[te + j].a = start_ez;
  }

  /* bug in <for from="" to=""> ? */
  nav_cube.nline_x--;
  nav_cube.nline_z--;

  return FALSE;
}

bool_t nav_cube_run(int8_t j, int8_t i,
                    uint8_t dest_b, uint8_t dest_e,
                    uint8_t src_b, uint8_t src_e)
{

  if (i > nav_cube.nline_x) {
    return FALSE;
  }
  if (j > nav_cube.nline_z) {
    return FALSE;
  }

  waypoints[dest_b].x = waypoints[src_b + i].x;
  waypoints[dest_b].y = waypoints[src_b + i].y;
  waypoints[dest_b].a = waypoints[src_b + i].a + j * nav_cube.grid_z;
  /* always keep at least security altitude */
  if (waypoints[dest_b].a < (ground_alt + SECURITY_HEIGHT)) {
    waypoints[dest_b].a = ground_alt + SECURITY_HEIGHT;
  }

  waypoints[dest_e].x = waypoints[src_e + i].x;
  waypoints[dest_e].y = waypoints[src_e + i].y;
  waypoints[dest_e].a = waypoints[src_e + i].a + j * nav_cube.grid_z;
  /* always keep at least security altitude */
  if (waypoints[dest_e].a < (ground_alt + SECURITY_HEIGHT)) {
    waypoints[dest_e].a = ground_alt + SECURITY_HEIGHT;
  }

  return FALSE;
}
