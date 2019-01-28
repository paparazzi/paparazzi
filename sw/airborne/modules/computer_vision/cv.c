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
 * @file modules/computer_vision/cv.c
 *
 * Computer vision framework for onboard processing
 */

#include <stdlib.h> // for malloc
#include <stdio.h>

#include "cv.h"
#include "rt_priority.h"


void cv_attach_listener(struct video_config_t *device, struct video_listener *new_listener);
int8_t cv_async_function(struct cv_async *async, struct image_t *img);
void *cv_async_thread(void *args);


static inline uint32_t timeval_diff(struct timeval *A, struct timeval *B)
{
  return (B->tv_sec - A->tv_sec) * 1000000 + (B->tv_usec - A->tv_usec);
}


struct video_listener *cv_add_to_device(struct video_config_t *device, cv_function func, uint16_t fps)
{
  // Create a new video listener
  struct video_listener *new_listener = malloc(sizeof(struct video_listener));

  // Assign function to listener
  new_listener->active = true;
  new_listener->func = func;
  new_listener->next = NULL;
  new_listener->async = NULL;
  new_listener->maximum_fps = fps;

  // Initialise the device that we want our function to use
  add_video_device(device);

  // Check if device already has a listener
  if (device->cv_listener == NULL) {
    // Add as first listener
    device->cv_listener = new_listener;
  } else {
    // Create pointer to first listener
    struct video_listener *listener = device->cv_listener;

    // Loop through linked list to last listener
    while (listener->next != NULL) {
      listener = listener->next;
    }

    // Add listener to end
    listener->next = new_listener;
  }

  return new_listener;
}


struct video_listener *cv_add_to_device_async(struct video_config_t *device, cv_function func, int nice_level,
    uint16_t fps)
{
  // Create a normal listener
  struct video_listener *listener = cv_add_to_device(device, func, fps);

  // Add asynchronous structure to override default synchronous behavior
  listener->async = malloc(sizeof(struct cv_async));
  listener->async->thread_priority = nice_level;

  // Explicitly mark img_copy as uninitialized
  listener->async->img_copy.buf = NULL;
  listener->async->img_copy.buf_size = 0;

  // Initialize mutex and condition variable
  pthread_mutex_init(&listener->async->img_mutex, NULL);
  pthread_cond_init(&listener->async->img_available, NULL);

  // Create new processing thread
  pthread_create(&listener->async->thread_id, NULL, cv_async_thread, listener);

#ifndef __APPLE__
  pthread_setname_np(listener->async->thread_id, "cv");
#endif

  return listener;
}


int8_t cv_async_function(struct cv_async *async, struct image_t *img)
{
  // If the previous image is not yet processed, return
  if (!async->img_processed || pthread_mutex_trylock(&async->img_mutex) != 0) {
    return -1;
  }

  // update image copy if input image size changed or not yet initialised
  if (async->img_copy.buf_size != img->buf_size) {
    if (async->img_copy.buf !=  NULL) {
      image_free(&async->img_copy);
    }
    image_create(&async->img_copy, img->w, img->h, img->type);
  }
#if CV_ALLOW_VIDEO_TO_CHANGE_SIZE
  // Note: must be enabled explicitly as not all modules may support this. (See issue #2187)
  if (img->buf_size > async->img_copy.buf_size) {
    image_free(&async->img_copy);
    image_create(&async->img_copy, img->w, img->h, img->type);
  }
#endif

  // Copy image
  image_copy(img, &async->img_copy);

  // Inform thread of new image
  async->img_processed = false;
  pthread_cond_signal(&async->img_available);
  pthread_mutex_unlock(&async->img_mutex);
  return 0;
}


void *cv_async_thread(void *args)
{
  struct video_listener *listener = args;
  struct cv_async *async = listener->async;
  async->thread_running = true;

  set_nice_level(async->thread_priority);

  // Request new image from video thread
  pthread_mutex_lock(&async->img_mutex);
  async->img_processed = true;

  while (async->thread_running) {
    // Wait for img available signal
    pthread_cond_wait(&async->img_available, &async->img_mutex);

    // Img might have been processed already (spurious wake-ups)
    if (async->img_processed) {
      continue;
    }

    // Execute vision function from this thread
    listener->func(&async->img_copy);

    // Mark image as processed
    async->img_processed = true;
  }

  pthread_mutex_unlock(&async->img_mutex);
  pthread_exit(NULL);
}


void cv_run_device(struct video_config_t *device, struct image_t *img)
{
  struct image_t *result;

  // Loop through computer vision pipeline
  for (struct video_listener *listener = device->cv_listener; listener != NULL; listener = listener->next) {
    // If the listener is not active, skip it
    if (!listener->active) {
      continue;
    }

    // If the desired frame time for this listener is not reached, skip it
    if (listener->maximum_fps > 0 && timeval_diff(&listener->ts, &img->ts) < (1000000 / listener->maximum_fps)) {
      continue;
    }

    if (listener->async != NULL) {
      // Send image to asynchronous thread, only update listener if successful
      if (!cv_async_function(listener->async, img)) {
        // Store timestamp
        listener->ts = img->ts;
      }
    } else {
      // Execute the cvFunction and catch result
      result = listener->func(img);

      // If result gives an image pointer, use it in the next stage
      if (result != NULL) {
        img = result;
      }
      // Store timestamp
      listener->ts = img->ts;
    }
  }
}
