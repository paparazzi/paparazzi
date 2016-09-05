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
#include "modules/sonar/sonar_bebop.h"

#include "modules/computer_vision/opencv_imav_landingpad.h"

#include "modules/computer_vision/marker/marker_checkers.h"

static bool SHOW_MARKER = true;

// General outputs
struct Marker marker;

// Helipad detection
static struct video_listener* helipad_listener;


static void geo_locate_marker(struct image_t* img) {
    // Obtain the relative pixel location (measured from center in body frame) rotated to vehicle reference frame
    struct FloatVect3 pixel_relative;
    pixel_relative.x = (float)(img->h / 2) - (float)marker.pixel.y;
    pixel_relative.y = (float)marker.pixel.x - (float)(img->w / 2);
    pixel_relative.z = 400.; // estimated focal length in px

    // Get the rotation measured at image capture
    struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);

    // Create a orientation representation
    struct FloatRMat ned_to_body;
    float_rmat_of_eulers(&ned_to_body, &pose.eulers);

    // Rotate the pixel vector from body frame to the north-east-down frame
    struct FloatVect3 geo_relative;
    float_rmat_transp_vmult(&geo_relative, &ned_to_body, &pixel_relative);

    // Divide by z-component to normalize the projection vector
    float zi = geo_relative.z;

    // Pointing up or horizontal -> no ground projection
    if (zi <= 0.) { return; }

    // Scale the parameters based on distance to ground and focal point
    struct NedCoor_f *pos = stateGetPositionNed_f();
    float agl = sonar_bebop.distance; // -pos->z

    geo_relative.x *= agl/zi;
    geo_relative.y *= agl/zi;
    geo_relative.z = agl;

    // TODO filter this location over time to reduce the jitter in output
    // TODO use difference in position as a velocity estimate along side opticflow in hff...

    // NED
    marker.geo_location.x = pos->x + geo_relative.x;
    marker.geo_location.y = pos->y + geo_relative.y;
    marker.geo_location.z = 0;

    //  OLD METHOD
//    struct camera_frame_t cam;
//    cam.px = marker.pixel.x;
//    cam.py = marker.pixel.y;
//    cam.f = 400; // Focal length [px]
//    cam.h = img->w; // Frame height [px]
//    cam.w = img->h; // Frame width [px]
//
//    georeference_project_target(&cam);
//    marker.geo_location.x = POS_FLOAT_OF_BFP(geo.target_abs.x);
//    marker.geo_location.y = POS_FLOAT_OF_BFP(geo.target_abs.y);
//    marker.geo_location.z = POS_FLOAT_OF_BFP(geo.target_abs.z);
    //fprintf(stderr, "[marker] found! %.3f, %.3f, %.3f, %.3f\n", geo_relative.x, geo_relative.y, marker.geo_location.x, marker.geo_location.y);
}

static struct image_t *detect_marker_checkers(struct image_t* img) {

    struct resultsc marker_checkers = opencv_detect_checkers((char*) img->buf, img->w, img->h);

    return NULL;
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

    if (helipad_marker.marker) {
        marker.detected = true;
        marker.pixel.x = helipad_marker.maxx;
        marker.pixel.y = helipad_marker.maxy;

        geo_locate_marker(img);
//        fprintf(stderr, "[marker] found! %i, %i\n", marker.pixel.x, marker.pixel.y);
    } else {
        marker.detected = false;

        //fprintf(stderr, "[marker] not found!\n");
    }
    return NULL;
}


static struct image_t *draw_target_marker(struct image_t* img)
{
    if (marker.detected && SHOW_MARKER) {
        struct point_t t = {marker.pixel.x, marker.pixel.y - 50},
                b = {marker.pixel.x, marker.pixel.y + 50},
                l = {marker.pixel.x - 50, marker.pixel.y},
                r = {marker.pixel.x + 50, marker.pixel.y};

        image_draw_line(img, &t, &b);
        image_draw_line(img, &l, &r);
    }

    return img;
}


void detector_init(void)
{
    // Add detection function to CV
//    helipad_listener = cv_add_to_device_async(&DETECTOR_CAMERA1, detect_helipad_marker, 5);
//    helipad_listener->maximum_fps = 10;

    init_detect_checkers();

    cv_add_to_device(&DETECTOR_CAMERA1, detect_marker_checkers);

    cv_add_to_device(&DETECTOR_CAMERA1, draw_target_marker);
}
