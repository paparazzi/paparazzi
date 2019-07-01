/*
 * Copyright (C) 2017 Tom van Dijk
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * Video thread dummy for simulation. *
 *
 * Keeps track of added devices, which can be referenced by simulation code
 * such as in simulator/nps/fdm_gazebo.c.
 */

// Own header
#include "video_thread_nps.h"
#include "video_thread.h"
#include "cv.h"
#include "lib/vision/image.h"

#include "modules/computer_vision/lib/v4l/v4l2.h"
#include "peripherals/video_device.h"

#include <stdio.h>

// Camera structs for use in modules.
// See boards/pc_sim.h
// Default values from ARDrone can be overwritten by simulator.
struct video_config_t front_camera = {
  .output_size = { .w = 1280, .h = 720 },
  .sensor_size = { .w = 1280, .h = 720 },
  .crop = { .x = 0, .y = 0, .w = 1280, .h = 720 },
  .dev_name = "front_camera",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 10,
  .filters = 0,
  .cv_listener = NULL,
  .fps = 0,
  .camera_intrinsics = {
    .focal_x = 300,
    .focal_y = 300,
    .center_x = 1280 / 2,
    .center_y = 720 / 2,
    .Dhane_k = 1
  }
};

struct video_config_t bottom_camera = {
  .output_size = { .w = 240, .h = 240 },
  .sensor_size = { .w = 320, .h = 240 },
  .crop = { .x = 40, .y = 0, .w = 240, .h = 240 },
  .dev_name = "bottom_camera",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 10,
  .filters = 0,
  .cv_listener = NULL,
  .fps = 0,
  .camera_intrinsics = {
    .focal_x = 350,
    .focal_y = 350,
    .center_x = 240 / 2,
    .center_y = 240 / 2,
    .Dhane_k = 1
  }
};

// Keep track of added devices.
struct video_config_t *cameras[VIDEO_THREAD_MAX_CAMERAS] = { NULL };

// All dummy functions
void video_thread_init(void)
{
}
void video_thread_periodic(void)
{

}

void video_thread_start(void)
{
}
void video_thread_stop(void)
{
}

/**
 *  Keep track of video devices added by modules.
 */
bool add_video_device(struct video_config_t *device)
{
  // Loop over camera array
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    // If device is already registered, break
    if (cameras[i] == device) {
      break;
    }
    // If camera slot is already used, continue
    if (cameras[i] != NULL) {
      continue;
    }
    // No initialization, should be handled by simulation!
    // Store device pointer
    cameras[i] = device;
    // Debug statement
    printf("[video_thread_nps] Added %s to camera array.\n",
           device->dev_name);
    // Successfully added
    return true;
  }
  // Camera array is full
  return false;
}
