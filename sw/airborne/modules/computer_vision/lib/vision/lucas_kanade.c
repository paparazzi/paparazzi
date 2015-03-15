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
 * @param[in] points_cnt The amount of points
 * @param[out] *new_points The new locations of the points
 * @param[out] *status Whether the point was tracked or not
 * @param[in] half_window_size Half the window size (in both x and y direction) to search inside
 * @param[in] max_iteration Maximum amount of iterations to find the new point
 * @param[in] step_threshold The threshold at which the iterations should stop
 */
void opticFlowLK(struct image_t *new_img, struct image_t *old_img, struct point_t *points, uint16_t points_cnt,
                struct point_t *new_points, bool_t *status, uint16_t half_window_size, uint8_t max_iterations, uint8_t step_threshold)
{
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


  // spatial resolution of flow is 1 / subpixel_factor (TODO: set ourself)
  uint16_t subpixel_factor = 10;

  // determine patch sizes and initialize neighborhoods
  uint16_t patch_size = 2 * half_window_size;
  uint32_t error_threshold = (25 * 25) * (patch_size * patch_size);
  uint16_t padded_patch_size = patch_size + 2;

  // Create the window images
  struct image_t window_I, window_J, window_DX, window_DY, window_diff;
  image_create(&window_I, padded_patch_size, padded_patch_size, IMAGE_GRAYSCALE);
  image_create(&window_J, patch_size, patch_size, IMAGE_GRAYSCALE);
  image_create(&window_DX, patch_size, patch_size, IMAGE_GRADIENT);
  image_create(&window_DY, patch_size, patch_size, IMAGE_GRADIENT);
  image_create(&window_diff, patch_size, patch_size, IMAGE_GRADIENT);

  for (uint16_t p = 0; p < points_cnt; p++) {
    // Update the status that the point isn't lost yet
    status[p] = TRUE;

    // If the pixel is outside ROI, do not track it
    if(points[p].x < half_window_size || (old_img->w - points[p].x) < half_window_size
      || points[p].y < half_window_size || (old_img->h - points[p].y) < half_window_size)
    {
      status[p] = FALSE;
      continue;
    }

    // Convert the point to a subpixel coordinate
    points[p].x *= subpixel_factor;
    points[p].y *= subpixel_factor;

    // (1) determine the subpixel neighborhood in the old image
    image_subpixel_window(old_img, &window_I, &points[p], subpixel_factor);

    // (2) get the x- and y- gradients
    image_gradients(&window_I, &window_DX, &window_DY);

    // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
    int32_t G[4];
    image_calculate_g(&window_DX, &window_DY, G);

    // calculate G's determinant in subpixel units:
    int32_t Det = (G[0] * G[3] - G[1] * G[2]) / subpixel_factor;

    // Check if the determinant is bigger than 1
    if (Det < 1) {
      status[p] = FALSE;
      continue;
    }

    // (4) iterate over taking steps in the image to minimize the error:
    memcpy(&new_points[p], &points[p], sizeof(struct point_t));
    for(uint8_t it = 0; it < max_iterations; it++)
    {
      // If the pixel is outside ROI, do not track it
      if(new_points[p].x/subpixel_factor < half_window_size || (old_img->w - new_points[p].x/subpixel_factor) < half_window_size
        || new_points[p].y/subpixel_factor < half_window_size || (old_img->h - new_points[p].y/subpixel_factor) < half_window_size)
      {
        status[p] = FALSE;
        break;
      }

      //     [a] get the subpixel neighborhood in the new image
      image_subpixel_window(&window_J, new_img, &new_points[p], subpixel_factor);

      //     [b] determine the image difference between the two neighborhoods
      uint32_t error = image_difference(&window_I, &window_J, &window_diff);
      if (error > error_threshold && it > max_iterations / 2) {
        status[p] = FALSE;
        break;
      }

      int32_t b_x = image_multiply(&window_diff, &window_DX, NULL) / 255;
      int32_t b_y = image_multiply(&window_diff, &window_DY, NULL) / 255;

      //     [d] calculate the additional flow step and possibly terminate the iteration
      int16_t step_x = (G[3] * b_x - G[1] * b_y) / Det;
      int16_t step_y = (G[0] * b_y - G[2] * b_x) / Det;
      new_points[p].x += step_x;
      new_points[p].y += step_y;

      // Check if we exceeded the treshold
      if(abs(step_x) < step_threshold && abs(step_y) < step_threshold)
        break;
    }

    // Convert the point back to the original coordinate (TODO: maybe round it as it is closer to the original)
    new_points[p].x /= subpixel_factor;
    new_points[p].y /= subpixel_factor;
    points[p].x /= subpixel_factor;
    points[p].y /= subpixel_factor;
  }

  // Free the images
  image_free(&window_I);
  image_free(&window_J);
  image_free(&window_DX);
  image_free(&window_DY);
  image_free(&window_diff);
}
