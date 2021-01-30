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

typedef struct image_t *(*cv_function)(struct image_t *img, uint8_t camera_id);

struct cv_async {
  pthread_t thread_id;
  volatile bool thread_running;
  volatile int thread_priority;
  pthread_mutex_t img_mutex;
  pthread_cond_t img_available;
  volatile bool img_processed;
  struct image_t img_copy;
};

struct video_listener {
  struct video_listener *next;
  struct cv_async *async;
  struct timeval ts;
  cv_function func;
  uint8_t id;

  // Can be set by user
  uint16_t maximum_fps;
  volatile bool active;
};

extern bool add_video_device(struct video_config_t *device);

extern struct video_listener *cv_add_to_device(struct video_config_t *device, cv_function func, uint16_t fps, uint8_t id);
extern struct video_listener *cv_add_to_device_async(struct video_config_t *device, cv_function func, int nice_level,
    uint16_t fps, uint8_t id);

extern void cv_run_device(struct video_config_t *device, struct image_t *img);

#endif /* CV_H_ */
