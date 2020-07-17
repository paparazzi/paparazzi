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



// Custom C headers to include
#include "modules/computer_vision/lib/vision/image.h" // Needed for imae_t struct type
#include "modules/wedgebug/wedgebug.h" // needed for calculating crop dimensions


// Global functions:

extern int save_image_gray(struct image_t *img, char *myString);
int save_image_color(struct image_t *img, char *myString);
int save_image_HM(struct image_t *img, char *myString, int const heatmap);
extern int SBM_OCV(struct image_t *img_disp, const struct image_t *img_left, const struct image_t *img_right,
                   const int ndisparities, const int SADWindowSize, const bool cropped);
extern int opening_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size,
                       const int iteration);
extern int closing_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size,
                       const int iteration);
extern int dilation_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size,
                        const int iteration);
extern int erosion_OCV(struct image_t *img_input, const struct image_t *img_output, const int SE_size,
                       const int iteration);
int sobel_OCV(struct image_t *img_input, const struct image_t *img_output, const int kernel_size, const int thr);




#ifdef __cplusplus
}
#endif

#endif  // WEDGEBUG_OPENCV_H
