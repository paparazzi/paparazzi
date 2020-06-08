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
 * @file modules/guidance/gvf_advanced/gvf_advanced.h
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 */

#ifndef GVF_ADVANCED_H
#define GVF_ADVANCED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_advanced/trajectories/gvf_advanced_3d_ellipse.h"


typedef struct {
    float w;
    float delta_T;
} gvf_advanced_con;

// Parameters for the trajectories
enum trajectories_advanced {
    ELLIPSE_3D = 0,
    NONE_ADVANCED = 255,
};

typedef struct {
    enum trajectories_advanced type;
    float p_advanced[16];
} gvf_advanced_tra;

extern gvf_advanced_tra gvf_advanced_trajectory;

// Init function
extern void gvf_advanced_init(void);

// 3D Ellipse
extern bool gvf_advanced_3D_ellipse(float, float, float, float, float, float);

#ifdef __cplusplus
}
#endif


#endif // GVF_ADVANCED_H
