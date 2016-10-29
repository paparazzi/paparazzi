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

/** \file gvf_sin.c
 *
 *  Guidance algorithm based on vector fields
 *  2D sinusoidal trajectory
 */


#include "subsystems/navigation/common_nav.h"
#include "gvf_sin.h"

void gvf_sin_info(float *phi, struct gvf_grad *grad,
        struct gvf_Hess *hess){

    struct EnuCoor_f *p = stateGetPositionEnu_f();
    float px = p->x;
    float py = p->y;
    float a = gvf_param[0];
    float b = gvf_param[1];
    float alpha = gvf_param[2];
    float w = gvf_param[3];
    float off = gvf_param[4];
    float A = gvf_param[5];

    // Phi(x,y)
    float xs =   (px-a)*sinf(alpha) - (py-b)*cosf(alpha);
    float ys =  -(px-a)*cosf(alpha) - (py-b)*sinf(alpha);

    // TODO Make it always in (-pi, pi] in an efficient way
    float ang = (w*xs + off);

    // Phi(x,y)
    *phi = ys - A*sinf(ang);

    // grad Phi
    grad->nx =  -cosf(alpha) - A*w*sinf(alpha)*cosf(ang);
    grad->ny =  -sinf(alpha) + A*w*cosf(alpha)*cosf(ang);

    // Hessian Phi
    hess->H11 =  -A*w*w*sinf(alpha)*sinf(alpha)*sinf(ang);
    hess->H12 =  -A*w*w*sinf(alpha)*cosf(alpha)*sinf(ang);
    hess->H21 =  -A*w*w*cosf(alpha)*sinf(alpha)*sinf(ang);
    hess->H22 =  -A*w*w*cosf(alpha)*cosf(alpha)*sinf(ang);
}
