/*
 * Copyright (C) 2023 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef GVF_PARAMETRIC_2D_BEZIER_SPLINES_H
#define GVF_PARAMETRIC_2D_BEZIER_SPLINES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/trajectories/gvf_param_traj.h"

typedef struct {
  float kx;
  float ky;
} gvf_par_2d_bezier_par;

extern gvf_par_2d_bezier_par gvf_parametric_2d_bezier_par;

// 2D BEZIER
extern bool nav_gvf_parametric_2D_bezier_run(void);
extern bool nav_gvf_parametric_2D_bezier_wp(uint8_t first_wp);

#ifdef __cplusplus
}
#endif

#endif // bezier splines
