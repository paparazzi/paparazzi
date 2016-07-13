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
 * @file "modules/computer_vision/marker/detector.c"
 */

#include <stdio.h>

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/marker/detector.h"

#include "modules/computer_vision/opencv_imav_landingpad.h"

// General outputs
volatile bool marker_detected;
int marker_pixel_x;
int marker_pixel_y;

// Helipad detection
static struct video_listener* helipad_listener;

// Function
static struct image_t *detect_marker(struct image_t* img)
{
    struct results helipad_marker = opencv_imav_landing(
            (char*) img->buf,
            img->w,
            img->h,
            2, //squares
            220, //binary threshold
            0); //modify image

    if (helipad_marker.MARKER) {
        marker_detected = true;
        marker_pixel_x = helipad_marker.maxx;
        marker_pixel_y = helipad_marker.maxy;

        fprintf(stderr, "[MARKER] found! %i, %i\n", marker_pixel_x, marker_pixel_y);
    } else {
        marker_detected = false;

        fprintf(stderr, "[MARKER] not found!\n");
    }

    return NULL;
}

void detector_init(void)
{
    // Add detection function to CV
    helipad_listener = cv_add_to_device_async(&DETECTOR_CAMERA1, detect_marker, 5);
    helipad_listener->maximum_fps = 10;
}