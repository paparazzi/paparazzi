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

/** \file gvf_line.c
 *
 *  Guidance algorithm based on vector fields
 *  2D straight line trajectory
 */


#include "subsystems/navigation/common_nav.h"
#include "gvf_line.h"

void gvf_line_info(float *phi, struct gvf_grad *grad,
        struct gvf_Hess *hess){

    struct EnuCoor_f *p = stateGetPositionEnu_f();
    float px = p->x;
    float py = p->y;
    float a = gvf_param[0];
    float b = gvf_param[1];
    float alpha = gvf_param[2];

    // Phi(x,y)
    *phi = -(px-a)*cosf(alpha) + (py-b)*sinf(alpha);

    // grad Phi
    grad->nx =  -cosf(alpha);
    grad->ny =   sinf(alpha);

    // Hessian Phi
    hess->H11 = 0;
    hess->H12 = 0;
    hess->H21 = 0;
    hess->H22 = 0;
}
