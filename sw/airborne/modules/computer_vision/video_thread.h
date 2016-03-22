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
 * @file modules/computer_vision/video_thread.h
 *
 * Start a Video thread and grab images
 *
 * Works on Linux platforms
 */

#ifndef VIDEO_THREAD_H
#define VIDEO_THREAD_H

#include "std.h"

// Main video_thread structure
struct video_thread_t {
  volatile bool is_running;   ///< When the device is running
  struct v4l2_device *dev;        ///< The V4L2 device that is used for the video stream
  uint8_t fps;                    ///< The amount of frames per second

  volatile bool take_shot;      ///< Wether to take an image
  uint16_t shot_number;           ///< The last shot number
};
extern struct video_thread_t video_thread;

// Module functions
extern void video_thread_init(void);
extern void video_thread_periodic(void); ///< A dummy for now
extern void video_thread_start(void);
extern void video_thread_stop(void);
extern void video_thread_take_shot(bool take);

#endif /* VIDEO_THREAD_H */

