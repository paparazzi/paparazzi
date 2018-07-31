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
 * @file modules/computer_vision/lib/vision/undistortion.c
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


// Own Header
#include "undistortion.h"
#include <math.h>

/**
 * Distort normalized image coordinates with the invertible Dhane method. This can be useful for undistorting an entire image.
 * We assume f, the focal length, to be 1 here. That is why it is missing from the below formula.
 * @param[in] x_n The undistorted normalized x coordinate
 * @param[in] y_n The undistorted normalized y coordinate
 * @param[in] k The single parameter of Dhane's model, typically found empirically by checking if straight lines in the world are straight in an undistorted image.
 * @param[out] *x_nd The distorted normalized x coordinate
 * @param[out] *y_nd The distorted normalized y coordinate
 * @return Whether the distortion was successful
 */
bool Dhane_distortion(float x_n, float y_n, float* x_nd, float* y_nd, float k) {
  float R = sqrtf(x_n*x_n + y_n*y_n);
  float r = tanf( asinf( (1.0f / k) * sinf( atanf( R ) ) ) );
  float reduction_factor = r/R;
  (*x_nd) = reduction_factor * x_n;
  (*y_nd) = reduction_factor * y_n;
  return true;
}

/**
 * Undistort distorted normalized image coordinates with the invertible Dhane method.
 * We assume f, the focal length, to be 1 here. That is why it is missing from the below formula.
 * @param[out] *x_n The undistorted normalized x coordinate
 * @param[out] *y_n The undistorted normalized y coordinate
 * @param[in] k The single parameter of Dhane's model, typically found empirically by checking if straight lines in the world are straight in an undistorted image.
 * @param[in] x_nd The distorted normalized x coordinate
 * @param[in] y_nd The distorted normalized y coordinate
 * @return Whether the distortion was successful
 */
bool Dhane_undistortion(float x_nd, float y_nd, float* x_n, float* y_n, float k) {
  float r = sqrtf( x_nd*x_nd + y_nd*y_nd );
  float inner_part = sinf( atanf( r ) ) * k;
  // we will take the asine of the inner part. It can happen that it is outside of [-1, 1], in which case, it would lead to an error.
  if(fabs(inner_part) > 0.9999) {
    return false;
  }

  float R = tanf( asinf( inner_part ) );
  float enlargement_factor = R / r;
  (*x_n) = enlargement_factor * x_nd;
  (*y_n) = enlargement_factor * y_nd;

  return true;
}

/**
 * Transform normalized coordinates to pixel coordinates.
 * @param[in] x_n The undistorted normalized x coordinate
 * @param[in] y_n The undistorted normalized y coordinate
 * @param[in] *K The camera calibration matrix, as a single array in row-major form, so K00, K01, K02, K10, K11, ...
 * @param[out] *x_p The pixel x coordinate
 * @param[out] *y_p The pixel y coordinate
 */
void normalized_to_pixels(float x_n_, float y_n_, float* x_p, float* y_p, const float* K) {
  (*x_p) = x_n_ * K[0] + K[2];
  (*y_p) = y_n_ * K[4] + K[5];
}

/**
 * Transform pixel coordinates to normalized coordinates.
 * @param[out] *x_n The undistorted normalized x coordinate
 * @param[out] *y_n The undistorted normalized y coordinate
 * @param[in] *K The camera calibration matrix, as a single array in row-major form, so K00, K01, K02, K10, K11, ...
 * @param[in] x_p The pixel x coordinate
 * @param[in] y_p The pixel y coordinate
 */
void pixels_to_normalized(float x_p, float y_p, float* x_n_, float* y_n_, const float* K) {
  (*x_n_) = (x_p - K[2]) / K[0];
  (*y_n_) = (y_p - K[5]) / K[4];
}

/**
 * Transform distorted pixel coordinates to normalized coordinates.
 * @param[out] *x_n The undistorted normalized x coordinate
 * @param[out] *y_n The undistorted normalized y coordinate
 * @param[in] k The single parameter of Dhane's model, typically found empirically by checking if straight lines in the world are straight in an undistorted image.
 * @param[in] *K The camera calibration matrix, as a single array in row-major form, so K00, K01, K02, K10, K11, ...
 * @param[in] x_pd The distorted pixel x coordinate
 * @param[in] y_pd The distorted pixel y coordinate
 * @return Whether the transformation was successful
 */
bool distorted_pixels_to_normalized_coords(float x_pd, float y_pd, float* x_n, float* y_n, float k, const float* K) {
  float x_nd, y_nd;
  pixels_to_normalized(x_pd, y_pd, &x_nd, &y_nd, K);
  bool success = Dhane_undistortion(x_nd, y_nd, x_n, y_n, k);
  return success;
}

/**
 * Transform normalized coordinates to distorted pixel coordinates.
 * @param[in] x_n The undistorted normalized x coordinate
 * @param[in] y_n The undistorted normalized y coordinate
 * @param[in] k The single parameter of Dhane's model, typically found empirically by checking if straight lines in the world are straight in an undistorted image.
 * @param[in] *K The camera calibration matrix, as a single array in row-major form, so K00, K01, K02, K10, K11, ...
 * @param[out] *x_pd The distorted pixel x coordinate
 * @param[out] *y_pd The distorted pixel y coordinate
 * @return Whether the transformation was successful
 */

bool normalized_coords_to_distorted_pixels(float x_n, float y_n, float *x_pd, float *y_pd, float k, const float* K) {
  float x_nd, y_nd;
  bool success = Dhane_distortion(x_n, y_n, &x_nd, &y_nd, k);
  if(!success) {
    return false;
  }
  else {
    normalized_to_pixels(x_nd, y_nd, x_pd, y_pd, K);
  }
  return success;
}
