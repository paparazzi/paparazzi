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

void cv_add_to_device(struct video_config_t *device, cvFunction func)
{
  // Initialise the device that we want our function to use
  add_video_device(device);

  // Create a new video listener
  struct video_listener *new_listener = malloc(sizeof(struct video_listener));

  // Assign function to listener
  new_listener->next = NULL;
  new_listener->func = func;

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

void cv_run_device(struct video_config_t *device, struct image_t *img,FloatEulers imageStartEulers,FloatEulers imageEndEulers)
{
  // For each function added to a device, run this function with the image that was taken
  struct video_listener *pointing_to = device->pointer_to_first_listener;

  // Loop through computer vision pipeline
  while (pointing_to != NULL) {
    // Execute the cvFunction and catch result
    struct image_t *result = pointing_to->func(img);

    // If result gives an image pointer, use it in the next stage
    if (result != NULL)
      img = result;

    // Move forward in the pipeline
    pointing_to = pointing_to->next;
  }
}
