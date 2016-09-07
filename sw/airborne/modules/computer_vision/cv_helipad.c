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
 * @file "modules/computer_vision/cv_helipad.c"
 * @author IMAV 2016
 * Module for detecting helipad marker in IMAV 2016
 */

#include "modules/computer_vision/cv_helipad.h"
#include "modules/computer_vision/opencv_imav_landingpad.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/imav2016markers.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/sonar/sonar_bebop.h"
#include <stdio.h>

// General outputs
pthread_mutex_t marker_mutex;
int MARKER;
int maxx;
int maxy;

// Colorfilter settings
int color_lum_min = 220;
int color_lum_max = 255;
int color_cb_min  = 0;
int color_cb_max  = 255;
int color_cr_min  = 0;
int color_cr_max  = 255;
int blob_threshold = 2500; // Reliable white detection

// Helipad detection
static struct video_listener* helipad_listener;
struct results helipad_marker;
struct results_color color_marker;

// Image-modification triggers
int modify_image_color   = FALSE;
int modify_image_helipad = TRUE;

// Marker detection
int squares = 4;
int med = 0;

// binary threshold
int bin_thresh = 230;

// Function
struct image_t *helipad_tracking_func(struct image_t* img);
struct image_t *helipad_tracking_func(struct image_t* img)
{
    /* TODO: Check if we really need blob locator. This may cause false positives.  */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COLORFILTER

    // Blob locator
    color_marker = locate_blob(img,
                               color_lum_min, color_lum_max,
                               color_cb_min, color_cb_max,
                               color_cr_min, color_cr_max,
                               blob_threshold);

    // Display the marker location and center-lines.
    if ((modify_image_color) && (color_marker.MARKER)) {
        int ti = color_marker.maxy - 50;
        int bi = color_marker.maxy + 50;
        struct point_t t = {color_marker.maxx, ti}, b = {color_marker.maxx, bi};

        int li = color_marker.maxx - 50;
        int ri = color_marker.maxx + 50;
        struct point_t l = {li, color_marker.maxy}, r = {ri, color_marker.maxy};

        image_draw_line(img, &t, &b);
        image_draw_line(img, &l, &r);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // OPENCV HELIPAD DETECTION

    // Change the threshold of valid squares depending on sonar altitude.
    if (sonar_bebop.distance < 1.2) {
        squares = 1;
    } else {
        squares = 2;
    }

    if (sonar_bebop.distance < 1.6) {
        bin_thresh = 230;
    } else {
        bin_thresh = 220;
    }


    if (img->type == IMAGE_YUV422) {
        // Call OpenCV (C++ from paparazzi C function)
        /* TODO: The image cannot be in grayscale and solve warnings */
        helipad_marker = opencv_imav_landing((char*) img->buf, img->w, img->h, squares, bin_thresh, modify_image_helipad);
    } else {
        helipad_marker.MARKER = 0;
    }

    pthread_mutex_lock(&marker_mutex);
    if (helipad_marker.MARKER) {
        MARKER = helipad_marker.MARKER;
        maxx   = helipad_marker.maxx;
        maxy   = helipad_marker.maxy;
    } else if ((color_marker.MARKER) && (sonar_bebop.distance > 1)) {
    //        MARKER = color_marker.MARKER;
    //        maxx   = color_marker.maxx;
    //        maxy   = color_marker.maxy;
    } else {
        MARKER = FALSE;
    }
    pthread_mutex_unlock(&marker_mutex);

    return NULL;
}

void helipad_init(void)
{
    // Add detection function to CV
    helipad_listener = cv_add_to_device_async(&HELIPAD_CAMERA, helipad_tracking_func, 5);
    helipad_listener->maximum_fps = 10;
}


int helipad_periodic(void) {
    /* currently no direct periodic functionality */
    return false;
}


int start_helipad(void) {
    helipad_listener->active=true;
    return false;
}


int stop_helipad(void) {
    helipad_listener->active=false;
    return false;
}