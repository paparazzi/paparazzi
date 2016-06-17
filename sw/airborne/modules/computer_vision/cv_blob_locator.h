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
 * @file "modules/computer_vision/cv_blob_locator.h"
 * @author C. De Wagter
 * Find a colored item and track its geo-location and update a waypoint to it
 */

#ifndef CV_BLOB_LOCATOR_H
#define CV_BLOB_LOCATOR_H

#include <stdint.h>

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern uint8_t cv_blob_locator_reset;
extern uint8_t cv_blob_locator_type;

extern int marker_size;
extern int geofilter_length;
extern int record_video;

extern void cv_blob_locator_init(void);
extern void cv_blob_locator_periodic(void);
extern void cv_blob_locator_event(void);
extern void cv_blob_locator_start(void);
extern void cv_blob_locator_stop(void);

#define cv_blob_locator_GeoReset(_v) {       \
    cv_blob_locator_start();                 \
  }

#define StartVision(X) { start_vision(); false; }
#define StartVisionLand(X) { start_vision_land(); false; }
#define StopVision(X) { stop_vision(); false; }


extern void start_vision(void);
extern void start_vision_land(void);
extern void stop_vision(void);


#endif

