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
 * @file modules/computer_vision/lib/vision/undistortion.h
 * @brief Functions for undistorting camera images.
 *
 * The lenses of most cameras distort the image, in the sense that it does not represent a linear projection of points in the world.
 * Specifically, straight lines in the world get curved in the image, as the lens compresses the data more and more towards the edges of the image.
 *
 * Reading:
 * 1. https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html (for normal cameras)
 * 2. https://docs.opencv.org/trunk/db/d58/group__calib3d__fisheye.html (for fisheye cameras)
 * 3. Dhane, P., Kutty, K., & Bangadkar, S. (2012). A generic non-linear method for fisheye correction. International Journal of Computer Applications, 51(10). (invertible fisheye model)
 *
 * Some definitions:
 * - x_n, y_n: Normalized coordinates, i.e., with no distortion, with (0,0) the center of the image, and x_n = X / Z
 * - x_nd, y_nd: Distorted normalized coordinates, i.e., with (0,0) the center of the image, but x_nd and y_nd are a nonlinear function of X/Z
 * - x_n_, y_n_: Normalized coordinates, whether distorted or undistorted.
 * - x_p, y_p: Pixel coordinates, with (0,0) the start of the image, and x_pd, y_pd in the range [0,width] and [0,height], respectively.
 * - k: distortion parameters
 * - K: camera calibration matrix, as a single array in row-major form, so K00, K01, K02, K10, K11, ...
 */



#ifndef UNDISTORTION_H
#define UNDISTORTION_H

#include "std.h"

// TODO: add other distortion models than just the Dhane one:
bool Dhane_distortion(float x_n, float y_n, float* x_nd, float* y_nd, float k);
bool Dhane_undistortion(float x_nd, float y_nd, float* x_n, float* y_n, float k);

void normalized_to_pixels(float x_n_, float y_n_, float* x_p, float* y_p, const float* K);
void pixels_to_normalized(float x_p, float y_p, float* x_n_, float* y_n_, const float* K);

bool distorted_pixels_to_normalized_coords(float x_pd, float y_pd, float* x_n, float* y_n, float k, const float* K);
bool normalized_coords_to_distorted_pixels(float x_n, float y_n, float *x_pd, float *y_pd, float k, const float* K);


#endif /* UNDISTORTION_H */
