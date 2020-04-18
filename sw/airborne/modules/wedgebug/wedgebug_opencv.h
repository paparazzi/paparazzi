/*
 * Copyright (C) Ralph Rudi schmidt <ralph.r.schmidt@outlook.com>

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
/** @file "modules/wedgebug/wedgebug_opencv.h"
 * @author Ralph Rudi schmidt <ralph.r.schmidt@outlook.com>
 * OpenCV functions to be used for the wedebug.
 */

#ifndef WEDGEBUG_OPENCV_H
#define WEDGEBUG_OPENCV_H





//
#ifdef __cplusplus
extern "C" {
#endif

#include "modules/computer_vision/lib/vision/image.h"
#include <stdint.h>
int SBM(struct image_t *left, struct image_t *right, struct image_t *matched, int ndisparities, int SADWindowSize);


int save_image_gray(void *img, int width, int height, char *myString);
int save_image_color(void *img, int width, int height, char *myString);
//int BM(void *img_left,void *img_right, void *img_output, int width, int height);

#ifdef __cplusplus
}
#endif

#endif  // WEDGEBUG_OPENCV_H
