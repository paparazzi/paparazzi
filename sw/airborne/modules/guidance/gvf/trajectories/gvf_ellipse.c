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

/** \file gvf_ellipse.c
 *
 *  Guidance algorithm based on vector fields
 *  2D Ellipse trajectory
 */


#include "subsystems/navigation/common_nav.h"
#include "gvf_ellipse.h"

#ifndef GVF_ELLIPSE_A
#define GVF_ELLIPSE_A 80
#endif

#ifndef GVF_ELLIPSE_B
#define GVF_ELLIPSE_B 80
#endif

#ifndef GVF_ELLIPSE_ALPHA
#define GVF_ELLIPSE_ALPHA 0
#endif

float gvf_ellipse_a = GVF_ELLIPSE_A;
float gvf_ellipse_b = GVF_ELLIPSE_B;
float gvf_ellipse_alpha = GVF_ELLIPSE_ALPHA;

void gvf_ellipse_info(float *phi, struct gvf_grad *grad, 
        struct gvf_Hess *hess){

    struct EnuCoor_f *p = stateGetPositionEnu_f();
    float px = p->x;
    float py = p->y;
    float wx = gvf_param[0];
    float wy = gvf_param[1];
    float a = gvf_param[2];
    float b = gvf_param[3];
    float alpha = gvf_param[4];

    // Phi(x,y)
    float xel = (px-wx)*cosf(alpha) - (py-wy)*sinf(alpha);
    float yel = (px-wx)*sinf(alpha) + (py-wy)*cosf(alpha);
    *phi = (xel/a)*(xel/a) + (yel/b)*(yel/b) - 1;

    // grad Phi
    grad->nx = (2*xel/(a*a))*cosf(alpha) + (2*yel/(b*b))*sinf(alpha);
    grad->ny = (2*yel/(b*b))*cosf(alpha) - (2*xel/(a*a))*sinf(alpha);

    // Hessian Phi
    hess->H11 = 2*(cosf(alpha)*cosf(alpha)/(a*a)
            + sinf(alpha)*sinf(alpha)/(b*b));
    hess->H12 = 2*sinf(alpha)*cosf(alpha)*(1/(b*b) - 1/(a*a));
    hess->H21 = hess->H12;
    hess->H22 = 2*(sinf(alpha)*sinf(alpha)/(a*a)
            + cosf(alpha)*cosf(alpha)/(b*b));
}
