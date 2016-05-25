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
 * @file modules/computer_vision/video_capture.c
 */


#include "modules/computer_vision/video_capture.h"
#include "modules/computer_vision/cv.h"

#define VIDEO_CAPTURE_JPEG_QUALITY 99

// Module settings
bool video_capture_take_shot = false;

// Forward function declarations
struct image_t *video_capture_func(struct image_t *img);


void video_capture_init(void) {
    cv_add_to_device(&VIDEO_CAPTURE_CAMERA, video_capture_func);
}


struct image_t *video_capture_func(struct image_t *img)
{
    if (video_capture_take_shot) {
        // TODO: save img to VIDEO_CAPTURE_PATH

        video_capture_take_shot = false;
    }

    return NULL;
}