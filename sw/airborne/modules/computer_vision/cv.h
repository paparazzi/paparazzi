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
 * @file modules/computer_vision/cv.h
 *
 * Computer vision framework for onboard processing
 */


#ifndef CV_H_
#define CV_H_

#include <pthread.h>

#include "std.h"
#include "peripherals/video_device.h"

#include BOARD_CONFIG

typedef struct image_t *(*cv_function)(struct image_t *img);

struct cv_async {
    pthread_t thread_id;
    volatile bool thread_running;
    pthread_mutex_t img_mutex;
    pthread_cond_t img_available;
    volatile bool img_processed;
    struct image_t img_copy;

    // Can be set by user
    int maximum_fps;
};

struct video_listener {
    struct video_listener *next;
    struct cv_async *async;
    cv_function func;
};

extern bool add_video_device(struct video_config_t *device);

// Adds a computer vision function to the video pipeline, setting asynchronous to true will spawn a thread
extern struct cv_async *cv_add_to_device(struct video_config_t *device, cv_function func, bool asynchronous);

extern void cv_run_device(struct video_config_t *device, struct image_t *img);

#endif /* CV_H_ */
