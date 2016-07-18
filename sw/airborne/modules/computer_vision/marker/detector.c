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

#include "state.h"
#include "math/pprz_orientation_conversion.h"

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/marker/detector.h"

#include "modules/computer_vision/cv_georeference.h"
#include "modules/pose_history/pose_history.h"

#include "modules/computer_vision/opencv_imav_landingpad.h"

static bool SHOW_MARKER = true;

// General outputs
struct marker MARKER;

// Helipad detection
static struct video_listener* helipad_listener;


static void geo_locate_marker(struct image_t* img) {
    // Obtain the relative pixel location (measured from center in body frame)
    struct FloatVect3 pixel_relative;
    pixel_relative.x = (img->h / 2) - MARKER.pixel.y;
    pixel_relative.y = MARKER.pixel.x - (img->w / 2);
    pixel_relative.z = 400; // estimated focal length

    // Get the rotation measured at image capture
    struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);

    // Create a orientation representation
    struct OrientationReps ned_to_body;
    orientationSetEulers_f(&ned_to_body, &pose.eulers);

    // Rotate the pixel vector from body frame to the north-east-down frame
    struct FloatVect3 geo_relative;
    float_rmat_transp_vmult(&geo_relative, orientationGetRMat_f(&ned_to_body), &pixel_relative);

    // target_l is now a scale-less [pix<<POS_FRAC] vector in LTP from the drone to the target
    // Divide by z-component to normalize the projection vector
    float zi = geo_relative.z;

    // Pointing up or horizontal -> no ground projection
    if (zi <= 0) { return; }

    // Multiply with height above ground
    struct NedCoor_f *pos = stateGetPositionNed_f();
    float zb = -pos->z; // Distance to target is equal to altitude
    geo_relative.x *= zb;
    geo_relative.y *= zb;

    // Divide by z-component
    geo_relative.x /= zi;
    geo_relative.y /= zi;
    geo_relative.z = zb;

    // NED
    MARKER.geo_location.x = pos->x + geo_relative.x;
    MARKER.geo_location.y = pos->y + geo_relative.y;
    MARKER.geo_location.z = 0;


    //  OLD METHOD
//    struct camera_frame_t cam;
//    cam.px = MARKER.pixel.x;
//    cam.py = MARKER.pixel.y;
//    cam.f = 400; // Focal length [px]
//    cam.h = img->w; // Frame height [px]
//    cam.w = img->h; // Frame width [px]
//
//    georeference_project_target(&cam);
//    MARKER.geo_location.x = POS_FLOAT_OF_BFP(geo.target_abs.x);
//    MARKER.geo_location.y = POS_FLOAT_OF_BFP(geo.target_abs.y);
//    MARKER.geo_location.z = POS_FLOAT_OF_BFP(geo.target_abs.z);

    fprintf(stderr, "[MARKER] found! %.3f, %.3f\n", MARKER.geo_location.x, MARKER.geo_location.y);
}


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

//        fprintf(stderr, "[MARKER] found! %i, %i\n", MARKER.pixel.x, MARKER.pixel.y);
    } else {
        MARKER.detected = false;

        fprintf(stderr, "[MARKER] not found!\n");
    }

    if (MARKER.detected) {
        geo_locate_marker(img);
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