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



#ifndef GVF_PARAMETRIC_2D_BEZIER_N_SEG
#define GVF_PARAMETRIC_2D_BEZIER_N_SEG 4
#endif


#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	float kx;
	float ky;
}gvf_par_2d_bezier_par;


// Cubic bezier
typedef struct{
	float p0[2];
	float p1[2];
	float p2[2];
	float p3[2];
}bezier_t;


extern gvf_par_2d_bezier_par gvf_parametric_2d_bezier_par;

extern void create_bezier_spline(bezier_t *bezier, float *px, float *py);
extern void gvf_parametric_2d_bezier_splines_info(bezier_t *bezier, float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd);

#ifdef __cplusplus
}
#endif





#endif // bezier splines
