/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/lib/vision/image.h
 * Image helper functions like resizing, color filter, converters...
 */

#ifndef _CV_LIB_VISION_IMAGE_H
#define _CV_LIB_VISION_IMAGE_H

#include "std.h"
#include <sys/time.h>
#include <state.h>

/* The different type of images we currently support */
enum image_type {
  IMAGE_YUV422,     ///< UYVY format (uint16 per pixel)
  IMAGE_GRAYSCALE,  ///< Grayscale image with only the Y part (uint8 per pixel)
  IMAGE_JPEG,       ///< An JPEG encoded image (not per pixel encoded)
  IMAGE_GRADIENT    ///< An image gradient (int16 per pixel)
};

/* Main image structure */
struct image_t {
  enum image_type type;   ///< The image type
  uint16_t w;             ///< Image width
  uint16_t h;             ///< Image height
  struct timeval ts;      ///< The timestamp of creation
  struct FloatEulers eulers;   ///< Euler Angles at time of image
  uint32_t pprz_ts;       ///< The timestamp in us since system startup

  uint8_t buf_idx;        ///< Buffer index for V4L2 freeing
  uint32_t buf_size;      ///< The buffer size
  void *buf;              ///< Image buffer (depending on the image_type)
};

/* Image point structure */
struct point_t {
  uint32_t x;             ///< The x coordinate of the point
  uint32_t y;             ///< The y coordinate of the point
  uint16_t count;         ///< Number of times the point has been tracked successfully
  uint16_t x_sub;         ///< The x subpixel coordinate of the point
  uint16_t y_sub;         ///< The y subpixel coordinate of the point
};

/* Vector structure for point differences */
struct flow_t {
  struct point_t pos;         ///< The original position the flow comes from
  int16_t flow_x;             ///< The x direction flow in subpixels
  int16_t flow_y;             ///< The y direction flow in subpixels
  uint32_t error;             ///< The matching error in the tracking process
};

/* Image size structure */
struct img_size_t {
  uint16_t w;     ///< The width
  uint16_t h;     ///< The height
};

/* Image crop structure */
struct crop_t {
  uint16_t x;    ///< Start position x (horizontal)
  uint16_t y;    ///< Start position y (vertical)
  uint16_t w;    ///< Width of the cropped area
  uint16_t h;    ///< height of the cropped area
};

/* Usefull image functions */
void image_add_border(struct image_t *input, struct image_t *output, uint8_t border_size);
void image_create(struct image_t *img, uint16_t width, uint16_t height, enum image_type type);
void image_free(struct image_t *img);
void image_copy(struct image_t *input, struct image_t *output);
void image_switch(struct image_t *a, struct image_t *b);
void image_to_grayscale(struct image_t *input, struct image_t *output);
uint16_t image_yuv422_colorfilt(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
                                uint8_t u_M, uint8_t v_m, uint8_t v_M);
int check_color_yuv422(struct image_t *im, int x, int y, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M);
void set_color_yuv422(struct image_t *im, int x, int y, uint8_t Y, uint8_t U, uint8_t V);
void image_yuv422_downsample(struct image_t *input, struct image_t *output, uint8_t downsample);
void image_subpixel_window(struct image_t *input, struct image_t *output, struct point_t *center,
                           uint32_t subpixel_factor, uint8_t border_size);
void image_gradients(struct image_t *input, struct image_t *dx, struct image_t *dy);
void image_calculate_g(struct image_t *dx, struct image_t *dy, int32_t *g);
uint32_t image_difference(struct image_t *img_a, struct image_t *img_b, struct image_t *diff);
int32_t image_multiply(struct image_t *img_a, struct image_t *img_b, struct image_t *mult);
void image_show_points(struct image_t *img, struct point_t *points, uint16_t points_cnt);
void image_show_points_color(struct image_t *img, struct point_t *points, uint16_t points_cnt, uint8_t *color);
void image_show_flow_color(struct image_t *img, struct flow_t *vectors, uint16_t points_cnt, uint8_t subpixel_factor,
                           const uint8_t *color, const uint8_t *bad_color);
void image_show_flow(struct image_t *img, struct flow_t *vectors, uint16_t points_cnt, uint8_t subpixel_factor);
void image_draw_crosshair(struct image_t *img, struct point_t *loc, const uint8_t *color, uint32_t size_crosshair);
void image_draw_rectangle(struct image_t *img, int x_min, int x_max, int y_min, int y_max, uint8_t *color);
void image_draw_line(struct image_t *img, struct point_t *from, struct point_t *to);
void image_draw_line_color(struct image_t *img, struct point_t *from, struct point_t *to, const uint8_t *color);
void pyramid_next_level(struct image_t *input, struct image_t *output, uint8_t border_size);
void pyramid_build(struct image_t *input, struct image_t *output_array, uint8_t pyr_level, uint16_t border_size);
void image_gradient_pixel(struct image_t *img, struct point_t *loc, int method, int *dx, int *dy);

#endif
