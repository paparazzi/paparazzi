/*
 * Copyright (C) 2015
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/blob/blob_finder.h
 *
 * Parse UYVY images and make a list of blobs of connected pixels
 */


#include "modules/computer_vision/lib/vision/image.h"


/* YUV Color Filter Parameters */
struct image_filter_t {
  uint8_t y_min;            ///< YUV color filter
  uint8_t y_max;
  uint8_t u_min;
  uint8_t u_max;
  uint8_t v_min;
  uint8_t v_max;
};

/* Blob object: connected pixels */
struct image_label_t {
  uint16_t id;              ///< Blob number
  uint8_t filter;           ///< Which filter triggered this blob

  uint16_t pixel_cnt;       ///< Number of pixels in the blob
  uint16_t x_min;           ///< Top left corner
  uint16_t y_min;
  uint32_t x_sum;           ///< Sum of all x coordinates (used to find center of gravity)
  uint32_t y_sum;

  struct point_t contour[512];
  uint16_t contour_cnt;

  uint16_t corners[4];
};

void image_labeling(struct image_t *input, struct image_t *output, struct image_filter_t *filters, uint8_t filters_cnt,
                    struct image_label_t *labels, uint16_t *labels_count);
