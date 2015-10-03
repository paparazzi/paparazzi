/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/bebop_front_camera.h
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Bebop platforms
 */

#ifndef BEBOP_FRONT_CAMERA_H
#define BEBOP_FRONT_CAMERA_H

#include "std.h"

// Main viewvideo structure
struct bebopfrontcamera_t {
  struct v4l2_device *dev;        ///< The V4L2 device that is used for the video stream
  uint8_t fps;                    ///< The amount of frames per second
  volatile bool_t take_shot;      ///< Wether to take an image
  uint16_t shot_number;           ///< The last shot number
  volatile bool_t is_streaming;   ///< When the device is streaming
  uint8_t downsize_factor;        ///< Downsize factor during the stream
  uint8_t quality_factor;         ///< Quality factor during the stream
  bool_t use_rtp;                 ///< Stream over RTP
};
extern struct bebopfrontcamera_t bebop_front_camera;

// Module functions
extern void bebop_front_camera_init(void);
extern void bebop_front_camera_periodic(void); ///< A dummy for now
extern void bebop_front_camera_start(void);
extern void bebop_front_camera_stop(void);
extern void bebop_front_camera_take_shot(bool_t take);

#endif /* BEBOP_FRONT_CAMERA_H */

