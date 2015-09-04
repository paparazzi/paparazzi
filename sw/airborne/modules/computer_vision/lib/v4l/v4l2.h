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
 * @file modules/computer_vision/lib/v4l/v4l2.h
 * Capture images from a V4L2 device (Video for Linux 2)
 */

#ifndef _CV_LIB_V4L2_H
#define _CV_LIB_V4L2_H

#include <linux/v4l2-subdev.h>
#include <pthread.h>
#include <sys/time.h>

#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"

#define V4L2_IMG_NONE 255  ///< There currently no image available

/* V4L2 memory mapped image buffer */
struct v4l2_img_buf {
  size_t length;              ///< The size of the buffer
  struct timeval timestamp;   ///< The time value of the image
  void *buf;                  ///< Pointer to the memory mapped buffer
};

/* V4L2 device */
struct v4l2_device {
  char *name;                       ///< The name of the device
  int fd;                           ///< The file pointer to the device
  pthread_t thread;                 ///< The thread that handles the images
  uint16_t w;                       ///< The width of the image
  uint16_t h;                       ///< The height of the image
  uint8_t buffers_cnt;              ///< The number of image buffers
  volatile uint8_t buffers_deq_idx; ///< The current dequeued index
  pthread_mutex_t mutex;            ///< Mutex lock for enqueue/dequeue of buffers (change the deq_idx)
  struct v4l2_img_buf *buffers;     ///< The memory mapped image buffers
};

/* External functions */
bool_t v4l2_init_subdev(char *subdev_name, uint8_t pad, uint8_t which, uint16_t code, uint16_t width, uint16_t height);
struct v4l2_device *v4l2_init(char *device_name, uint16_t width, uint16_t height, uint8_t buffers_cnt,
                              uint32_t _pixelformat);
void v4l2_image_get(struct v4l2_device *dev, struct image_t *img);
bool_t v4l2_image_get_nonblock(struct v4l2_device *dev, struct image_t *img);
void v4l2_image_free(struct v4l2_device *dev, struct image_t *img);
bool_t v4l2_start_capture(struct v4l2_device *dev);
bool_t v4l2_stop_capture(struct v4l2_device *dev);
void v4l2_close(struct v4l2_device *dev);

#endif /* _CV_LIB_V4L2_H */
