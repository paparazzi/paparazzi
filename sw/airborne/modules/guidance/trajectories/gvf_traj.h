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

#ifndef GVF_TRAJ_H
#define GVF_TRAJ_H

#include "modules/guidance/gvf_common.h"

enum trajectories {
  LINE = 0,
  ELLIPSE,
  SIN,
  NONE = 255,
};

typedef struct {
  enum trajectories type;
  float p[16];
  int p_len;
} gvf_tra;

struct gvf_grad {
  float nx;
  float ny;
  float nz;
};

struct gvf_Hess {
  float H11;
  float H12;
  float H13;
  float H21;
  float H22;
  float H23;
  float H31;
  float H32;
  float H33;
};

/** @typedef gvf_seg
* @brief Struct employed by the LINE trajectory for the special case of tracking
a segment, which is described by the coordinates x1, y1, x2, y2
* @param seg Tracking a segment or not
* @param x1 coordinate w.r.t. HOME
* @param y1 coordinate w.r.t. HOME
* @param x2 coordinate w.r.t. HOME
* @param y2 coordinate w.r.t. HOME
*/
typedef struct {
  int seg;
  float x1;
  float y1;
  float x2;
  float y2;
} gvf_seg;

extern gvf_tra gvf_trajectory;
extern gvf_seg gvf_segment;

/** ------------------------------------------------------------------------ **/

extern void gvf_line_info(float *phi, struct gvf_grad *, struct gvf_Hess *);
extern void gvf_ellipse_info(float *phi, struct gvf_grad *, struct gvf_Hess *);
extern void gvf_sin_info(float *phi, struct gvf_grad *, struct gvf_Hess *);

#endif // GVF_TRAJ_H