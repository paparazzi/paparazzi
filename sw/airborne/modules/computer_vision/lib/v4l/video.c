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
 * @file modules/computer_vision/lib/v4l/v4l.c
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

#include "video.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))
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
      return (void *) -1;
    }
    else if(sr == 0) {
      printf("[v4l2-capture] Select timeout on %s\n", dev->name);
      //continue;
      dev->thread = (pthread_t) NULL;
      return (void *) -2;
    }

    // Dequeue a buffer
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(dev->fd, VIDIOC_DQBUF, &buf) < 0) {
      printf("[v4l2-capture] Dequeue of buffer failed for %s.\n", dev->name);
      dev->thread = (pthread_t) NULL;
      return (void *) -3;
    }
    assert(buf.index < dev->buffers_cnt);

    //printf("Got image %d %d\n",  buf.timestamp.tv_sec, buf.timestamp.tv_usec);

    // Check if the current image is being processed
    struct v4l2_img_buf *prev_img = &dev->buffers[dev->buffers_deq_idx];
    if (dev->buffers_deq_idx != 255 && pthread_mutex_trylock(&prev_img->mutex) == 0) {
      // Change the current dequeued index
      dev->buffers_deq_idx = buf.index;

      // Enqueue the previous buffer
      buf.index = prev_img->idx;
      if (ioctl(dev->fd, VIDIOC_QBUF, &buf) < 0) {
        printf("[v4l2-capture] Could not enqueue %d for %s\n", prev_img->idx, dev->name);
      }
      pthread_mutex_unlock(&prev_img->mutex);
    }
    // Image is being processed so enqueue is done after processing
    else {
      // Change the current index
      dev->buffers_deq_idx = buf.index;
      //printf("[v4l2-capture] No enqueue, keep image and buffers_deq_idx: %d\n", dev->buffers_deq_idx);
    }
  }
  return (void *)0;
}

/**
 * Initialize a V4L2(Video for Linux 2) device
 * The device name should be something like "/dev/video1"
 * When the width and height are 0 the width and height of the device is queried
 * The buffer_cnt are the amount of buffers used in memory mapping
 * Not that you need to close this device at the end of you program!
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
  // Try to get access to the current image
  uint16_t deq_idx = dev->buffers_deq_idx;
  while (TRUE) {
    if(deq_idx != 255 && pthread_mutex_trylock(&dev->buffers[deq_idx].mutex) == 0)
      break;

    deq_idx = dev->buffers_deq_idx;
    usleep(1);
  }
  return &dev->buffers[deq_idx];
}

/**
 * Get the latest image and lock it (Thread safe, NON BLOCKING)
 * This function returns NULL if it can't get access to the current image.
 * Make sure you free the image after processing!
 */
struct v4l2_img_buf *v4l2_image_get_nonblock(struct v4l2_device *dev) {
  struct v4l2_img_buf *img_buf = &dev->buffers[dev->buffers_deq_idx];
  if (dev->buffers_deq_idx == 255 || pthread_mutex_trylock(&img_buf->mutex) != 0) {
    return NULL;
  }
  return img_buf;
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

  //RACE CONDITION!!
  if(dev->buffers_deq_idx == img_buf->idx)
    dev->buffers_deq_idx = 255;

  // Unlock the buffer
  pthread_mutex_unlock(&img_buf->mutex);
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
  dev->buffers_deq_idx = 255; // Set none to dequeued
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
    printf("[v4l2] Could not start stream of %s\n", dev->name);
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
    pthread_mutex_lock(&dev->buffers[i].mutex);
    if (munmap(dev->buffers[i].buf, dev->buffers[i].length) < 0) {
      printf("[v4l2] Could not unmap buffer %d for %s\n", i, dev->name);
    }
    pthread_mutex_unlock(&dev->buffers[i].mutex);
  }

  // Close the file pointer and free all memory
  close(dev->fd);
  free(dev->name);
  free(dev);
}
