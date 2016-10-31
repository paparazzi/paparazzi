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

/** \file gvf_sin.h
 *
 *  Guidance algorithm based on vector fields
 *  2D sinusoidal trajectory
 */

#ifndef GVF_SIN_H
#define GVF_SIN_H

#include "modules/guidance/gvf/gvf.h"

typedef struct {
  float alpha;
  float w;
  float off;
  float A;
} gvf_s_par;

extern gvf_s_par gvf_sin_par;

extern void gvf_sin_info(float *phi, struct gvf_grad *, struct gvf_Hess *);

#endif // GVF_SIN_H
