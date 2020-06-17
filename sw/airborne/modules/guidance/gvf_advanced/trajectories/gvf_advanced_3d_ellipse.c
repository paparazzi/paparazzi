/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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

/**
 * @file modules/guidance/gvf_advanced/trajectories/gvf_advanced_3d_ellipse.c
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 * 
 * 3D ellipse (intersection between a cylinder and a tilted plane)
 */

#include "gvf_advanced_3d_ellipse.h"

/*! Default gain kx for the 3d ellipse trajectory */
#ifndef GVF_ADVANCED_3D_ELLIPSE_KX
#define GVF_ADVANCED_3D_ELLIPSE_KX 0.004
#endif

/*! Default gain ky for the 3d ellipse trajectory */
#ifndef GVF_ADVANCED_3D_ELLIPSE_KY
#define GVF_ADVANCED_3D_ELLIPSE_KY 0.004
#endif

/*! Default gain kz for the 3d ellipse trajectory */
#ifndef GVF_ADVANCED_3D_ELLIPSE_KZ
#define GVF_ADVANCED_3D_ELLIPSE_KZ 0.09
#endif

/*! Default gain kpsi for the 3d ellipse trajectory */
#ifndef GVF_ADVANCED_3D_ELLIPSE_KPSI
#define GVF_ADVANCED_3D_ELLIPSE_KPSI 1
#endif

/*! Default radius of the cylinder */
#ifndef GVF_ADVANCED_3D_ELLIPSE_R
#define GVF_ADVANCED_3D_ELLIPSE_R 80
#endif

/*! Default highest point for the ellipse trajectory */
#ifndef GVF_ADVANCED_3D_ELLIPSE_ZL
#define GVF_ADVANCED_3D_ELLIPSE_ZL 40
#endif

/*! Default highest point for the ellipse trajectory */
#ifndef GVF_ADVANCED_3D_ELLIPSE_ZH
#define GVF_ADVANCED_3D_ELLIPSE_ZH 40
#endif

/*! Default orientation in degrees for the lowest point of the ellipse */
#ifndef GVF_ADVANCED_3D_ELLIPSE_ALPHA
#define GVF_ADVANCED_3D_ELLIPSE_ALPHA 0
#endif

gvf_adv_3d_ell_par gvf_advanced_3d_ellipse_par = {GVF_ADVANCED_3D_ELLIPSE_KX,
GVF_ADVANCED_3D_ELLIPSE_KY, GVF_ADVANCED_3D_ELLIPSE_KZ, GVF_ADVANCED_3D_ELLIPSE_KPSI, GVF_ADVANCED_3D_ELLIPSE_R, GVF_ADVANCED_3D_ELLIPSE_ZL, GVF_ADVANCED_3D_ELLIPSE_ZH, GVF_ADVANCED_3D_ELLIPSE_ALPHA};
