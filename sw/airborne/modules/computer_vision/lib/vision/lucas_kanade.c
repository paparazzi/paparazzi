/*
 * Copyright (C) 2014 G. de Croon
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/lib/vision/lucas_kanade.c
 * @brief efficient fixed-point optical-flow calculation
 *
 * - Initial fixed-point C implementation by G. de Croon
 * - Algorithm: Lucas-Kanade by Yves Bouguet
 * - Publication: http://robots.stanford.edu/cs223b04/algo_tracking.pdf
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "lucas_kanade.h"


/**
 * Compute the optical flow of several points using the Lucas-Kanade algorithm by Yves Bouguet
 * The initial fixed-point implementation is doen by G. de Croon and is adapted by
 * Freek van Tienen for the implementation in Paparazzi.
 * @param[in] *new_img The newest grayscale image (TODO: fix YUV422 support)
 * @param[in] *old_img The old grayscale image (TODO: fix YUV422 support)
 * @param[in] *points Points to start tracking from
 * @param[in,out] points_cnt The amount of points and it returns the amount of points tracked
 * @param[in] half_window_size Half the window size (in both x and y direction) to search inside
 * @param[in] subpixel_factor The subpixel factor which calculations should be based on
 * @param[in] max_iterations Maximum amount of iterations to find the new point
 * @param[in] step_threshold The threshold at which the iterations should stop
 * @param[in] max_points The maximum amount of points to track, we skip x points and then take a point.
 * @return The vectors from the original *points in subpixels
 */
struct flow_t *opticFlowLK(struct image_t *new_img, struct image_t *old_img, struct point_t *points, uint16_t *points_cnt,
                           uint16_t half_window_size, uint16_t subpixel_factor, uint8_t max_iterations, uint8_t step_threshold, uint16_t max_points) {
  // A straightforward one-level implementation of Lucas-Kanade.
  // For all points:
  // (1) determine the subpixel neighborhood in the old image
  // (2) get the x- and y- gradients
  // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
  // (4) iterate over taking steps in the image to minimize the error:
  //     [a] get the subpixel neighborhood in the new image
  //     [b] determine the image difference between the two neighborhoods
  //     [c] calculate the 'b'-vector
  //     [d] calculate the additional flow step and possibly terminate the iteration

  // Allocate some memory for returning the vectors
  struct flow_t *vectors = malloc(sizeof(struct flow_t) * max_points);
  uint16_t new_p = 0;
  uint16_t points_orig = *points_cnt;
  *points_cnt = 0;

  // determine patch sizes and initialize neighborhoods
  uint16_t patch_size = 2 * half_window_size;
  uint32_t error_threshold = (25 * 25) *(patch_size *patch_size);
  uint16_t padded_patch_size = patch_size + 2;

  // Create the window images
  struct image_t window_I, window_J, window_DX, window_DY, window_diff;
  image_create(&window_I, padded_patch_size, padded_patch_size, IMAGE_GRAYSCALE);
  image_create(&window_J, patch_size, patch_size, IMAGE_GRAYSCALE);
  image_create(&window_DX, patch_size, patch_size, IMAGE_GRADIENT);
  image_create(&window_DY, patch_size, patch_size, IMAGE_GRADIENT);
  image_create(&window_diff, patch_size, patch_size, IMAGE_GRADIENT);

  // Calculate the amount of points to skip
  float skip_points = (points_orig > max_points) ? points_orig / max_points : 1;

  // Go trough all points
  for (uint16_t i = 0; i < max_points && i < points_orig; i++) {
    uint16_t p = i * skip_points;

    // If the pixel is outside ROI, do not track it
    if (points[p].x < half_window_size || (old_img->w - points[p].x) < half_window_size
        || points[p].y < half_window_size || (old_img->h - points[p].y) < half_window_size) {
      continue;
    }

    // Convert the point to a subpixel coordinate
    vectors[new_p].pos.x = points[p].x * subpixel_factor;
    vectors[new_p].pos.y = points[p].y * subpixel_factor;
    vectors[new_p].flow_x = 0;
    vectors[new_p].flow_y = 0;

    // (1) determine the subpixel neighborhood in the old image
    image_subpixel_window(old_img, &window_I, &vectors[new_p].pos, subpixel_factor);

    // (2) get the x- and y- gradients
    image_gradients(&window_I, &window_DX, &window_DY);

    // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
    int32_t G[4];
    image_calculate_g(&window_DX, &window_DY, G);

    // calculate G's determinant in subpixel units:
    int32_t Det = (G[0] * G[3] - G[1] * G[2]) / subpixel_factor;

    // Check if the determinant is bigger than 1
    if (Det < 1) {
      continue;
    }

    // a * (Ax - Bx) + (1-a) * (Ax+1 - Bx+1)
    // a * Ax - a * Bx + (1-a) * Ax+1 - (1-a) * Bx+1
    // (a * Ax + (1-a) * Ax+1)  - (a * Bx + (1-a) * Bx+1)

    // (4) iterate over taking steps in the image to minimize the error:
    bool_t tracked = TRUE;
    for (uint8_t it = 0; it < max_iterations; it++) {
      struct point_t new_point =  {
        vectors[new_p].pos.x + vectors[new_p].flow_x,
        vectors[new_p].pos.y + vectors[new_p].flow_y
      };
      // If the pixel is outside ROI, do not track it
      if (new_point.x / subpixel_factor < half_window_size || (old_img->w - new_point.x / subpixel_factor) < half_window_size
          || new_point.y / subpixel_factor < half_window_size || (old_img->h - new_point.y / subpixel_factor) < half_window_size) {
        tracked = FALSE;
        break;
      }

      //     [a] get the subpixel neighborhood in the new image
      image_subpixel_window(new_img, &window_J, &new_point, subpixel_factor);

      //     [b] determine the image difference between the two neighborhoods
      uint32_t error = image_difference(&window_I, &window_J, &window_diff);
      if (error > error_threshold && it > max_iterations / 2) {
        tracked = FALSE;
        break;
      }

      int32_t b_x = image_multiply(&window_diff, &window_DX, NULL) / 255;
      int32_t b_y = image_multiply(&window_diff, &window_DY, NULL) / 255;

      //     [d] calculate the additional flow step and possibly terminate the iteration
      int16_t step_x = (G[3] * b_x - G[1] * b_y) / Det;
      int16_t step_y = (G[0] * b_y - G[2] * b_x) / Det;
      vectors[new_p].flow_x += step_x;
      vectors[new_p].flow_y += step_y;

      // Check if we exceeded the treshold
      if ((abs(step_x) + abs(step_y)) < step_threshold) {
        break;
      }
    }

    // If we tracked the point we update the index and the count
    if (tracked) {
      new_p++;
      (*points_cnt)++;
    }
  }

  // Free the images
  image_free(&window_I);
  image_free(&window_J);
  image_free(&window_DX);
  image_free(&window_DY);
  image_free(&window_diff);

  // Return the vectors
  return vectors;
}
