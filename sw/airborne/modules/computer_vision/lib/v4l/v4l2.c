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
 * @file modules/computer_vision/lib/v4l/v4l2.c
 * Capture images from a V4L2 device (Video for Linux 2)
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <pthread.h>

#include "v4l2.h"

#define CLEAR(x) memset(&(x), 0, sizeof (x))
static void *v4l2_capture_thread(void *data);

/**
 * The main capturing thread
 * This thread handles the queue and dequeue of buffers, to make sure only the latest
 * image buffer is preserved for image processing.
 */
static void *v4l2_capture_thread(void *data)
{
  struct v4l2_device *dev = (struct v4l2_device *)data;
  struct v4l2_buffer buf;
  struct timeval tv;
  fd_set fds;

  while (TRUE) {
    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    // Set the timeout to 2 seconds
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    // Wait until an image was taken, with a timeout of tv
    int sr = select(dev->fd + 1, &fds, NULL, NULL, &tv);
    if (sr < 0) {
      // Was interrupted by a signal
      if (EINTR == errno) { continue; }
      printf("[v4l2-capture] Select error %d on %s: %s\n", errno, dev->name, strerror(errno));
      dev->thread = (pthread_t) NULL;
      return (void *) - 1;
    } else if (sr == 0) {
      printf("[v4l2-capture] Select timeout on %s\n", dev->name);
      //continue;
      dev->thread = (pthread_t) NULL;
      return (void *) - 2;
    }

    // Dequeue a buffer
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(dev->fd, VIDIOC_DQBUF, &buf) < 0) {
      printf("[v4l2-capture] Dequeue of buffer failed for %s.\n", dev->name);
      dev->thread = (pthread_t) NULL;
      return (void *) - 3;
    }
    assert(buf.index < dev->buffers_cnt);

    // Update the dequeued id
    // We need lock because between setting prev_idx and updating the deq_idx the deq_idx could change
    pthread_mutex_lock(&dev->mutex);
    uint16_t prev_idx = dev->buffers_deq_idx;
    dev->buffers_deq_idx = buf.index;
    pthread_mutex_unlock(&dev->mutex);

    // Enqueue the previous image if not empty
    if (prev_idx != V4L2_IMG_NONE) {
      // Enqueue the previous buffer
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = prev_idx;
      if (ioctl(dev->fd, VIDIOC_QBUF, &buf) < 0) {
        printf("[v4l2-capture] Could not enqueue %d for %s\n", prev_idx, dev->name);
      }
    }

  }
  return (void *)0;
}

/**
 * Initialize a V4L2 subdevice.
 * The subdevice name should be something like '/dev/v4l-subdev0'
 * The pad and which indicate the way the subdevice should communicate
 * with the real device. Which pad it should take.
 * Code should be something like V4L2_MBUS_FMT_UYVY8_2X8. See the V4l2
 * manual for available codes.
 * Width and height are the amount of pixels this subdevice must cover.
 */
bool_t v4l2_init_subdev(char *subdev_name, uint8_t pad, uint8_t which, uint16_t code, uint16_t width, uint16_t height)
{
  struct v4l2_subdev_format sfmt;
  CLEAR(sfmt);

  // Try to open the subdevice
  int fd = open(subdev_name, O_RDWR, 0);
  if (fd < 0) {
    printf("[v4l2] Cannot open subdevice '%s': %d, %s\n", subdev_name, errno, strerror(errno));
    return FALSE;
  }

  // Try to get the subdevice data format settings
  if (ioctl(fd, VIDIOC_SUBDEV_G_FMT, &sfmt) < 0) {
    printf("[v4l2] Could not get subdevice data format settings of %s\n", subdev_name);
    close(fd);
    return FALSE;
  }

  // Set the new settings
  sfmt.pad = pad;
  sfmt.which = which;
  sfmt.format.width = width;
  sfmt.format.height = height;
  sfmt.format.code = code;
  sfmt.format.field = V4L2_FIELD_NONE;
  sfmt.format.colorspace = 1;

  if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sfmt) < 0) {
    printf("[v4l2] Could not set subdevice data format settings of %s\n", subdev_name);
    close(fd);
    return FALSE;
  }

  // Close the device
  close(fd);
  return TRUE;
}

/**
 * Initialize a V4L2(Video for Linux 2) device
 * The device name should be something like "/dev/video1"
 * The subdevice name can be empty if there is no subdevice
 * The buffer_cnt are the amount of buffers used in memory mapping
 * Note that you need to close this device at the end of you program!
 */
struct v4l2_device *v4l2_init(char *device_name, uint16_t width, uint16_t height, uint8_t buffers_cnt) {
  uint8_t i;
  struct v4l2_capability cap;
  struct v4l2_format fmt;
  struct v4l2_requestbuffers req;
  CLEAR(cap);
  CLEAR(fmt);
  CLEAR(req);

  // Try to open the device
  int fd = open(device_name, O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    printf("[v4l2] Cannot open '%s': %d, %s\n", device_name, errno, strerror(errno));
    return NULL;
  }

  // Try to fetch the capabilities of the V4L2 device
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
    printf("[v4l2] %s is no V4L2 device\n", device_name);
    close(fd);
    return NULL;
  }

  // Check if the device is capable of capturing and streaming
  if (!(cap.capabilities &V4L2_CAP_VIDEO_CAPTURE)) {
    printf("[v4l2] %s is no V4L2 video capturing device\n", device_name);
    close(fd);
    return NULL;
  }
  if (!(cap.capabilities &V4L2_CAP_STREAMING)) {
    printf("[v4l2] %s isn't capable of streaming (TODO: support reading)\n", device_name);
    close(fd);
    return NULL;
  }

  // TODO: Read video cropping and scaling information VIDIOC_CROPCAP

  // Set the format settings
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    printf("[v4l2] Could not set data format settings of %s\n", device_name);
    close(fd);
    return NULL;
  }

  // Request MMAP buffers
  req.count = buffers_cnt;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    printf("[v4l2] %s Does not support memory mapping\n", device_name);
    close(fd);
    return NULL;
  }

  // Allocate memory for the memory mapped buffers
  struct v4l2_img_buf *buffers = calloc(req.count, sizeof(struct v4l2_img_buf));
  if (buffers == NULL) {
    printf("[v4l2] Not enough memory for %s to initialize %d MMAP buffers\n", device_name, req.count);
    close(fd);
    return NULL;
  }

  // Go trough the buffers and initialize them
  for (i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf;
    CLEAR(buf);

    // Request the buffer information
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
      printf("[v4l2] Querying buffer %d from %s failed\n", i, device_name);
      free(buffers);
      close(fd);
      return NULL;
    }

    //  Map the buffer
    buffers[i].idx = i;
    buffers[i].length = buf.length;
    buffers[i].buf = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (MAP_FAILED == buffers[i].buf) {
      printf("[v4l2] Mapping buffer %d with length %d from %s failed\n", i, buf.length, device_name);
      free(buffers);
      close(fd);
      return NULL;
    }
  }

  // Create the device only when everything succeeded
  struct v4l2_device *dev = (struct v4l2_device *)malloc(sizeof(struct v4l2_device));
  CLEAR(*dev);
  dev->name = strdup(device_name); // NOTE: needs to be freed
  dev->fd = fd;
  dev->w = width;
  dev->h = height;
  dev->buffers_cnt = req.count;
  dev->buffers = buffers;
  return dev;
}

/**
 * Get the latest image buffer and lock it (Thread safe, BLOCKING)
 * This functions blocks until image access is granted. This should not take that long, because
 * it is only locked while enqueueing an image.
 * Make sure you free the image after processing!
 */
struct v4l2_img_buf *v4l2_image_get(struct v4l2_device *dev) {
  uint16_t img_idx = V4L2_IMG_NONE;

  // Continu to wait for an image
  while (img_idx == V4L2_IMG_NONE) {
    // We first check if the deq_idx is ok, this reduces the amount of locks
    if (dev->buffers_deq_idx != V4L2_IMG_NONE) {
      pthread_mutex_lock(&dev->mutex);

      // We need to check it here again, because it could be changed
      if (dev->buffers_deq_idx != V4L2_IMG_NONE) {
        img_idx = dev->buffers_deq_idx;
        dev->buffers_deq_idx = V4L2_IMG_NONE;
      }

      pthread_mutex_unlock(&dev->mutex);
    }
  }

  // Rreturn the image
  return &dev->buffers[img_idx];
}

/**
 * Get the latest image and lock it (Thread safe, NON BLOCKING)
 * This function returns NULL if it can't get access to the current image.
 * Make sure you free the image after processing!
 */
struct v4l2_img_buf *v4l2_image_get_nonblock(struct v4l2_device *dev) {
  uint16_t img_idx = V4L2_IMG_NONE;

  // Try to get the current image
  pthread_mutex_lock(&dev->mutex);
  if (dev->buffers_deq_idx != V4L2_IMG_NONE) {
    img_idx = dev->buffers_deq_idx;
    dev->buffers_deq_idx = V4L2_IMG_NONE;
  }
  pthread_mutex_unlock(&dev->mutex);

  // Check if we really got an image
  if (img_idx == V4L2_IMG_NONE) {
    return NULL;
  } else {
    return &dev->buffers[img_idx];
  }
}

/**
 * Free the image and enqueue the buffer (Thread safe)
 * This must be done after processing the image, because else all buffers are locked
 */
void v4l2_image_free(struct v4l2_device *dev, struct v4l2_img_buf *img_buf)
{
  struct v4l2_buffer buf;

  // Enqueue the buffer
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = img_buf->idx;
  if (ioctl(dev->fd, VIDIOC_QBUF, &buf) < 0) {
    printf("[v4l2] Could not enqueue %d for %s\n", img_buf->idx, dev->name);
  }
}

/**
 * Start capturing images in streaming mode (Thread safe)
 * Returns true when successfully started capturing. Not that it also returns
 * FALSE when it already is in capturing mode.
 */
bool_t v4l2_start_capture(struct v4l2_device *dev)
{
  uint8_t i;
  enum v4l2_buf_type type;

  // Check if not already running
  if (dev->thread != (pthread_t)NULL) {
    printf("[v4l2] There is already a capturing thread running for %s\n", dev->name);
    return FALSE;
  }

  // Enqueue all buffers
  dev->buffers_deq_idx = V4L2_IMG_NONE;
  for (i = 0; i < dev->buffers_cnt; ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(dev->fd, VIDIOC_QBUF, &buf) < 0) {
      printf("[v4l2] Could not enqueue buffer %d during start capture for %s\n", i, dev->name);
      return FALSE;
    }
  }

  // Start the stream
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(dev->fd, VIDIOC_STREAMON, &type) < 0) {
    printf("[v4l2] Could not start stream of %s, %d %s\n", dev->name, errno, strerror(errno));
    return FALSE;
  }

  //Start the capturing thread
  int rc = pthread_create(&dev->thread, NULL, v4l2_capture_thread, dev);
  if (rc < 0) {
    printf("[v4l2] Could not start capturing thread for %s (return code: %d)\n", dev->name, rc);

    // Stop the stream
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(dev->fd, VIDIOC_STREAMOFF, &type) < 0) {
      printf("[v4l2] Could not stop stream of %s\n", dev->name);
    }

    // Reset the thread
    dev->thread = (pthread_t) NULL;
    return FALSE;
  }

  return TRUE;
}

/**
 * Stop capturing of the image stream (Thread safe)
 * Returns TRUE if it successfully stopped capturing. Note that it also returns FALSE
 * when the capturing is already stopped. This function is blocking until capturing
 * thread is closed.
 */
bool_t v4l2_stop_capture(struct v4l2_device *dev)
{
  enum v4l2_buf_type type;

  // First check if still running
  if (dev->thread == (pthread_t) NULL) {
    printf("[v4l2] Already stopped capture for %s\n", dev->name);
    return FALSE;
  }

  // Stop the stream
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(dev->fd, VIDIOC_STREAMOFF, &type) < 0) {
    printf("[v4l2] Could not stop stream of %s\n", dev->name);
    return FALSE;
  }

  // Stop the thread
  if (pthread_cancel(dev->thread) < 0) {
    printf("[v4l2] Could not cancel thread for %s\n", dev->name);
    return FALSE;
  }

  // Wait for the thread to be finished
  pthread_join(dev->thread, NULL);
  dev->thread = (pthread_t) NULL;
  return TRUE;
}

/**
 * Close the V4L2 device (Thread safe)
 * This needs to be preformed to clean up all the buffers and close the device.
 * Note that this also stops the capturing if it is still capturing.
 */
void v4l2_close(struct v4l2_device *dev)
{
  uint8_t i;

  // Stop capturing (ignore result as it may already be stopped)
  v4l2_stop_capture(dev);

  // Unmap all buffers
  for (i = 0; i < dev->buffers_cnt; ++i) {
    if (munmap(dev->buffers[i].buf, dev->buffers[i].length) < 0) {
      printf("[v4l2] Could not unmap buffer %d for %s\n", i, dev->name);
    }
  }

  // Close the file pointer and free all memory
  close(dev->fd);
  free(dev->name);
  free(dev);
}
