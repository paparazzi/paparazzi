/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_georeference.h"
 * @author C. De Wagter
 * Geo-reference computer vision detections
 */

#ifndef CV_GEOREFERENCE_H
#define CV_GEOREFERENCE_H

#include "std.h"
#include <stdint.h>

extern int32_t focus_length;

extern void georeference_init(void);
extern void georeference_run(void);

struct camera_frame_t {
  int32_t w;     ///< Frame width [px]
  int32_t h;     ///< Frame height [px]
  int32_t f;     ///< Camera Focal length in [px]
  int32_t px;    ///< Target pixel coordinate (left = 0)
  int32_t py;    ///< Target pixel coordinate (top = 0)
};

void georeference_project(struct camera_frame_t *tar, int wp);
void georeference_filter(bool kalman, int wp, int length);


#endif

