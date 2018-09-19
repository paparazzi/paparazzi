/*
 * Copyright (C) Guido de Croon, 2018
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/lib/vision/PnP_AHRS.h
 * @brief Functions for solving a perspective-n-point problem, using the AHRS to get the relevant angles.
 *
 * Solves a perspective-n-point problem while using the attitude estimates of the AHRS. Using the AHRS makes the solution more robust
 * than a purely maximal likelihood estimate from a standard PnP solver.
 *
 * The module needs to get information on:
 * 1. The real-world coordinates of the points.
 * 2. The image coordinates of the points.
 * 3. The camera parameters for undistortion purposes.
 * 4. The rotation of the camera to the drone body.
 *
 */

#ifndef PNP_AHRS_H
#define PNP_AHRS_H

#include "std.h"
#include <math.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"
#include "peripherals/video_device.h"
#include "modules/computer_vision/lib/vision/undistortion.h"

// Get the world position of the camera, given image coordinates and corresponding world corners.
struct FloatVect3 get_world_position_from_image_points(int *x_corners, int *y_corners, struct FloatVect3 *world_corners,
    int n_corners,
    struct camera_intrinsics_t cam_intrinsics, struct FloatEulers cam_body);
#endif /* PNP_AHRS_H */
