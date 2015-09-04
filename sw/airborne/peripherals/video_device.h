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
 * @brief v4l2 video device settings interface
 * Works on Linux platforms
 */

#ifndef VIDEO_DEVICE_H
#define VIDEO_DEVICE_H

#include "std.h"
#include "modules/computer_vision/lib/v4l/v4l2.h"

/** V4L2 device settings */
struct video_device_t {
  int w;              ///< Width
  int h;              ///< Height
  char* dev_name;     ///< path to device
  char* subdev_name;  ///< path to sub device
  uint32_t format;    ///< Video format
  void* filters;      ///< filters to use
};


#endif /* VIDEO_DEVICE_H */
