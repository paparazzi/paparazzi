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
 * @file modules/computer_vision/detect_window.h
 *
 * Detect a bright region surrounded by dark or viceversa - sometimes this corresponds to a window.
 */


#ifndef WINDOW_DETECTION_MODULE
#define WINDOW_DETECTION_MODULE

#define MODE_DARK 0
#define MODE_BRIGHT 1

#include "inttypes.h"

extern void detect_window_init(void);
extern struct image_t* detect_window(struct image_t *img, uint8_t camera_id);

uint16_t detect_window_sizes(uint8_t *in, uint32_t image_width, uint32_t image_height, uint16_t *coordinate,
                             uint32_t *integral_image, uint8_t MODE);
uint16_t detect_window_one_size(uint8_t *in, uint32_t image_width, uint32_t image_height, uint16_t *coordinate,
                                uint16_t *size, uint8_t calculate_integral_image, uint32_t *integral_image, uint8_t MODE);
uint16_t detect_escape(uint8_t *in, uint32_t image_width, uint32_t image_height, uint16_t *escape_coordinate,
                       uint32_t *integral_image, uint8_t n_cells);
void get_integral_image(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image);
uint32_t get_sum_disparities(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                             uint32_t image_width, uint32_t image_height);
uint32_t get_avg_disparity(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                           uint32_t image_width, uint32_t image_height);
uint16_t get_window_response(uint16_t x, uint16_t y, uint16_t feature_size, uint16_t border, uint32_t *integral_image,
                             uint16_t image_width, uint16_t image_height, uint16_t px_inner, uint16_t px_border, uint8_t MODE);
uint16_t get_border_response(uint16_t x, uint16_t y, uint16_t feature_size, uint16_t window_size, uint16_t border_size,
                             uint32_t *integral_image, uint16_t image_width, uint16_t image_height, uint16_t px_inner, uint16_t px_outer);
void filter_bad_pixels(uint8_t *in, uint32_t image_width, uint32_t image_height);
void transform_illuminance_image(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height, uint8_t n_bits,
                                 uint8_t bright_win);

#endif /* WINDOW_DETECTION_MODULE */
