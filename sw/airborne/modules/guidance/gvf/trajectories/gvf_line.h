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

/** \file gvf_line.h
 *
 *  Guidance algorithm based on vector fields
 *  2D straight line trajectory
 */

#ifndef GVF_LINE_H
#define GVF_LINE_H

#include "modules/guidance/gvf/gvf.h"

typedef struct {
  float ke;
  float kn;
  float heading;
} gvf_li_par;

typedef struct {
  float d1;
  float d2;
} gvf_seg_par;

extern gvf_li_par gvf_line_par;
extern gvf_seg_par gvf_segment_par;

extern void gvf_line_info(float *phi, struct gvf_grad *, struct gvf_Hess *);

#endif // GVF_LINE_H
