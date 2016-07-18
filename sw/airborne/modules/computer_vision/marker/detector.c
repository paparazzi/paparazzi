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

#include "modules/computer_vision/cv_georeference.h"

#include "modules/computer_vision/opencv_imav_landingpad.h"

static bool SHOW_MARKER = true;

// General outputs
struct marker MARKER;

// Helipad detection
static struct video_listener* helipad_listener;

// Function
static struct image_t *detect_helipad_marker(struct image_t* img)
{
    struct results helipad_marker = opencv_imav_landing(
            (char*) img->buf,
            img->w,
            img->h,
            2, //squares
            220, //binary threshold
            0); //modify image

    if (helipad_marker.MARKER) {
        MARKER.detected = true;
        MARKER.pixel.x = helipad_marker.maxx;
        MARKER.pixel.y = helipad_marker.maxy;

        struct camera_frame_t cam;
        cam.px = MARKER.pixel.x;
        cam.py = MARKER.pixel.y;
        cam.f = 400; // Focal length [px]
        cam.h = img->w; // Frame height [px]
        cam.w = img->h; // Frame width [px]

        georeference_project_target(&cam);
        MARKER.geo_location.x = POS_FLOAT_OF_BFP(geo.target_abs.x);
        MARKER.geo_location.y = POS_FLOAT_OF_BFP(geo.target_abs.y);
        MARKER.geo_location.z = POS_FLOAT_OF_BFP(geo.target_abs.z);

//        fprintf(stderr, "[MARKER] found! %i, %i\n", MARKER.pixel.x, MARKER.pixel.y);
//        fprintf(stderr, "[MARKER] found! %.3f, %.3f\n", MARKER.geo_location.x, MARKER.geo_location.y);
    } else {
        MARKER.detected = false;

//        fprintf(stderr, "[MARKER] not found!\n");
    }

    return NULL;
}

static struct image_t *draw_target_marker(struct image_t* img)
{
    if (MARKER.detected && SHOW_MARKER) {
        struct point_t t = {MARKER.pixel.x, MARKER.pixel.y - 50},
                b = {MARKER.pixel.x, MARKER.pixel.y + 50},
                l = {MARKER.pixel.x - 50, MARKER.pixel.y},
                r = {MARKER.pixel.x + 50, MARKER.pixel.y};

        image_draw_line(img, &t, &b);
        image_draw_line(img, &l, &r);
    }

    return img;
}


void detector_init(void)
{
    // Add detection function to CV
    helipad_listener = cv_add_to_device_async(&DETECTOR_CAMERA1, detect_helipad_marker, 5);
    helipad_listener->maximum_fps = 10;

    cv_add_to_device(&DETECTOR_CAMERA1, draw_target_marker);
}