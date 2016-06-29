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

#include "cv.h"
#include <stdlib.h> // for malloc
#include <stdio.h>

void cv_attach_listener(struct video_config_t *device, struct video_listener *new_listener);
void cv_async_function(struct cv_async *async, struct image_t *img);
void *cv_async_thread(void *args);


void cv_attach_listener(struct video_config_t *device, struct video_listener *new_listener)
{
  // Initialise the device that we want our function to use
  add_video_device(device);

  // Check if device already has a listener
  if (device->pointer_to_first_listener == NULL) {
    // Add as first listener
    device->pointer_to_first_listener = new_listener;
  } else {
    // Create pointer to first listener
    struct video_listener *listener = device->pointer_to_first_listener;

    // Loop through linked list to last listener
    while (listener->next != NULL)
      listener = listener->next;

    // Add listener to end
    listener->next = new_listener;
  }
}


struct cv_async *cv_add_to_device(struct video_config_t *device, cv_function func, bool synchronous) {
  // Create a new video listener
  struct video_listener *listener = malloc(sizeof(struct video_listener));

  listener->next = NULL;
  listener->func = func;
  listener->async = NULL;

  if (!synchronous) {
    listener->async = malloc(sizeof(struct cv_async));

    // Explicitly mark img_copy as uninitialized
    listener->async->img_copy.buf_size = 0;

    // Initialize mutex and condition variable
    pthread_mutex_init(&listener->async->img_mutex, NULL);
    pthread_cond_init(&listener->async->img_available, NULL);

    // Create new processing thread
    pthread_create(&listener->async->thread_id, NULL, cv_async_thread, listener);
  }

  // Attach listener to device
  cv_attach_listener(device, listener);

  // Return async struct
  return listener->async;
}


void cv_async_function(struct cv_async *async, struct image_t *img) {

  fprintf(stderr, "[CV-ASYNC] Running async func.\n");

  // If image is not yet processed by thread, return
  if (pthread_mutex_trylock(&async->img_mutex) != 0 || !async->img_processed) {
    return;
  }

  // If the image has not been initialized, do it
  if (async->img_copy.buf_size == 0) {
    image_create(&(async->img_copy), img->w, img->h, img->type);

    fprintf(stderr, "[CV-ASYNC] created image.\n");
  }

  fprintf(stderr, "[CV-ASYNC] copying image.\n");


  // Copy image
  image_copy(img, &async->img_copy);

  fprintf(stderr, "[CV-ASYNC] copied image.\n");

  // Inform thread of new image
  async->img_processed = false;
  pthread_cond_signal(&async->img_available);
  pthread_mutex_unlock(&async->img_mutex);
}


void *cv_async_thread(void *args) {
  struct video_listener *listener = args;
  struct cv_async *async = listener->async;

  // Request new image from video thread
  pthread_mutex_lock(&async->img_mutex);
  async->img_processed = true;

  fprintf(stderr, "[CV-ASYNC] Running thread.\n");

  // TODO: add while condition
  while (true) {
    // Wait for img available signal
    pthread_cond_wait(&async->img_available, &async->img_mutex);

    fprintf(stderr, "[CV-ASYNC] Got condition signal.\n");

    // If image is processed already (spurious wake-up)
    if (async->img_processed) {
      continue;
    }

    // Execute vision function from this thread
    listener->func(&async->img_copy);

    // Request new image
    async->img_processed = true;
  }

  pthread_mutex_unlock(&async->img_mutex);

  pthread_exit(NULL);
}


void cv_run_device(struct video_config_t *device, struct image_t *img)
{
  // For each function added to a device, run this function with the image that was taken
  struct video_listener *pointing_to = device->pointer_to_first_listener;
  struct image_t *result;

  // Loop through computer vision pipeline
  while (pointing_to != NULL) {

    if (pointing_to->async != NULL) {
      // Send image to asynchronous thread
      cv_async_function(pointing_to->async, img);
    } else {
      // Execute the cvFunction and catch result
      result = pointing_to->func(img);

      // If result gives an image pointer, use it in the next stage
      if (result != NULL)
        img = result;
    }

    // Move forward in the pipeline
    pointing_to = pointing_to->next;
  }
}
