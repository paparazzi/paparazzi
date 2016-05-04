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
#include "video_thread.h"
void cv_add_to_device(struct video_config_t *device, cvFunction func)
{
  // Initialise the device that we want our function to use
  video_thread_initialise_device(device);

  // Check if we already have this listener for this device
  if (device->pointerToFirstListener == NULL) {
    struct video_listener *newListener = malloc(sizeof(struct video_listener));
    newListener->next = NULL;
    newListener->func = func;
    device->pointerToFirstListener = newListener;
  } else {
    struct video_listener *pointingTo = device->pointerToFirstListener;
    while (pointingTo->next != NULL) {
      pointingTo = pointingTo->next;
    }

    // The device is not yet sending the image to this function. Add it
    struct video_listener *newListener = malloc(sizeof(struct video_listener));
    newListener->next = NULL;
    newListener->func = func;
    pointingTo->next = newListener;
  }
}

void cv_run_device(struct video_config_t *device, struct image_t *img)
{
  // For each function added to a device, run this function with the image that was taken
  struct video_listener *pointingTo = device->pointerToFirstListener;
  while (pointingTo != NULL) {
    pointingTo->func(img);
    pointingTo = pointingTo->next;
  }

}
