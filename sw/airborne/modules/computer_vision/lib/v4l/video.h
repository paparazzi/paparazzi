/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com
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
 * @file modules/computer_vision/lib/v4l/v4l.h
 * Capture images from a V4L2 device (Video for Linux 2)
 */

#ifndef _V4L2_H
#define _V4L2_H

#include "std.h"

/* V4L2 memory mapped image buffer */
struct v4l2_img_buf {
  uint8_t idx;            //< The index of the buffer
  pthread_mutex_t mutex;  //< Mutex lock of an image (the image is being processed)
  size_t length;          //< The size of the buffer
  void *buf;              //< Pointer to the memory mapped buffer
};

/* V4L2 device */
struct v4l2_device {
  char *name;                     //< The name of the device
  int fd;                         //< The file pointer to the device
  pthread_t thread;               //< The thread that handles the images
  uint16_t w;                     //< The width of the image
  uint16_t h;                     //< The height of the image
  uint8_t buffers_cnt;            //< The number of image buffers
  uint8_t buffers_deq_idx;        //< The current dequeued index
  struct v4l2_img_buf *buffers;  //< The memory mapped image buffers
};

/* External functions */
struct v4l2_device *v4l2_init(char *device_name, uint16_t width, uint16_t height, uint8_t buffers_cnt);
struct v4l2_img_buf *v4l2_image_get(struct v4l2_device *dev);
struct v4l2_img_buf *v4l2_image_get_nonblock(struct v4l2_device *dev);
void v4l2_image_free(struct v4l2_device *dev, struct v4l2_img_buf *img_buf);
bool_t v4l2_start_capture(struct v4l2_device *dev);
bool_t v4l2_stop_capture(struct v4l2_device *dev);
void v4l2_close(struct v4l2_device *dev);

#endif /* _V4L2_H */
