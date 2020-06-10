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

#define GVF_ADVANCED_GRAVITY 9.806

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/guidance/gvf_advanced/trajectories/gvf_advanced_3d_ellipse.h"

/** @typedef gvf_advanced_con
* @brief Control parameters for the GVF_ADVANCED
* @param w Virtual coordinate from the parametrization of the trajectory
* @param delta_T Time between iterations needed for integrating w
* @param s Defines the direction to be tracked. It takes the values -1 or 1.
*/
typedef struct {
    float w;
    float delta_T;
    int8_t s;
} gvf_advanced_con;

extern gvf_advanced_con gvf_advanced_control;

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
extern bool gvf_advanced_3D_ellipse_XY(float, float, float, float, float, float);
extern bool gvf_advanced_3D_ellipse_wp(uint8_t, float, float, float, float);

#ifdef __cplusplus
}
#endif


#endif // GVF_ADVANCED_H
