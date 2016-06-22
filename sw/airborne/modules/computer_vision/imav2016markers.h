/*
 * Copyright (C) IMAV 2016
 *
 * This file is part of paparazzi
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
 * @file "modules/computer_vision/imav2016markers.h"
 * @author IMAV 2016
 */

#ifndef CV_MARKERS_H
#define CV_MARKERS_H

#include "std.h"
#include <stdint.h>
#include "math/pprz_algebra_int.h"

/* Structure with information about colorfilter */
struct colorfilter_t {
    float x_pos;
    float y_pos;
    uint32_t cnt;
};

struct colorfilter_t colorfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
                                            uint8_t u_M, uint8_t v_m, uint8_t v_M, uint8_t mod);
struct colorfilter_t grayfilter(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t mod);

/* Structure with information about blob_locator */
struct results_color {
    int maxx;
    int maxy;
    int MARKER;
};

struct results_color locate_blob(struct image_t *input,
                                 int y_m, int y_M,
                                 int u_m, int u_M,
                                 int v_m, int v_M,
                                 int threshold);

/* Coordinates structure */
struct centroid_t {
    float x;     ///< The x coordinate of the point
    float y;     ///< The y coordinate of the point
};

extern void georeference_init(void);

struct camera_frame_t {
  int32_t w;     ///< Frame width [px]
  int32_t h;     ///< Frame height [px]
  int32_t f;     ///< Camera Focal length in [px]
  int32_t px;    ///< Target pixel coordinate (left = 0)
  int32_t py;    ///< Target pixel coordinate (top = 0)
};

struct centroid_t georeference_project(struct camera_frame_t *tar);

#endif