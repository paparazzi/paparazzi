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
#include "math/pprz_geodetic_int.h"

struct georeference_filter_t {
    struct NedCoor_i x;          ///< Target
    struct NedCoor_i v;          ///< Target Velocity
    int32_t P;                    ///< Covariance/Average-count
};

struct georeference_t {
    struct Int32Vect3 target_p;   ///< Target in pixels, with z being the focal length in pixels, in camera frame x=up,y=right,out
    struct Int32Vect3 target_rel;    ///< Relative position to target
    struct NedCoor_i target_abs;    ///< Absolute position to target NED frame

    struct georeference_filter_t filter;  ///< Filter waypoint location
};

extern int32_t focal_length;
extern struct georeference_t geo;

extern void georeference_init(void);
extern void georeference_run(void);

struct camera_frame_t {
  int32_t w;     ///< Frame width [px]
  int32_t h;     ///< Frame height [px]
  int32_t f;     ///< Camera Focal length in [px]
  int32_t px;    ///< Target pixel coordinate (left = 0)
  int32_t py;    ///< Target pixel coordinate (top = 0)
};

/** Get geolocation of target relative to the vehicle and in absolute position
 * and update waypoint with that absolute position
 * @param tar target location in image frame with (0,0) in top left of image
 * @param wp waypoint to update
 */
void georeference_project(struct camera_frame_t *tar, int wp);

/** Get geolocation of target relative to the vehicle and in absolute position
 * @param tar target location in image frame with (0,0) in top left of image
 */
void georeference_project_target(struct camera_frame_t *tar);

/** Update filter of georeference and move waypoint accordingly
 * @param kalman 0 moving average filter, 1 kalman filter. NOTE Kalman filter not yet functional
 * @param wp waypoint to be updated
 * @param length Number of points for moving average filter
 */
void georeference_filter(bool kalman, int wp, int length);

/** update filter of georeference
 * @param kalman 0 moving average filter, 1 kalman filter
 * @param length Number of points for moving average filter
 */
void georeference_filter_target(bool kalman, int length);


#endif

