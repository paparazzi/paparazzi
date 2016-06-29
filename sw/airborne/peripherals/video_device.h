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
 * @file peripherals/video_device.h
 */

#ifndef VIDEO_DEVICE_H
#define VIDEO_DEVICE_H

#include <stdbool.h>
#include <inttypes.h>
#include "modules/computer_vision/lib/vision/image.h"

#define VIDEO_FILTER_DEBAYER 0x01

// Main video_thread structure
struct video_thread_t {
  volatile bool is_running;   ///< When the device is running
  struct v4l2_device *dev;        ///< The V4L2 device that is used for the video stream
};

/** V4L2 device settings */
struct video_config_t {
  int w;              ///< Width
  int h;              ///< Height
  char *dev_name;     ///< path to device
  char *subdev_name;  ///< path to sub device
  uint32_t format;    ///< Video format
  uint8_t buf_cnt;    ///< Amount of V4L2 video device buffers
  uint8_t filters;    ///< filters to use (bitfield with VIDEO_FILTER_x)
  struct video_thread_t thread; ///< Information about the thread this camera is running on
  void *pointer_to_first_listener; ///< The first listener in the linked list for this video device
  int fps;
};
extern struct video_config_t dummy_camera;


#endif
