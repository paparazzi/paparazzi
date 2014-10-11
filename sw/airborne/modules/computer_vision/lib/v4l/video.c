/*
    video.c - video driver

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <pthread.h>

#include "video.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))


pthread_t video_thread;
void *video_thread_main(void* data);
void *video_thread_main(void* data)
{
  struct vid_struct* vid = (struct vid_struct*)data;
  printf("video_thread_main started\n");
  while (1) {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(vid->fd, &fds);

    tv.tv_sec = 2;
    tv.tv_usec = 0;
    r = select(vid->fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
      if (EINTR == errno) continue;
      printf("select err\n");
    }

    if (0 == r) {
      fprintf(stderr, "select timeout\n");
      return 0;
    }

    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(vid->fd, VIDIOC_DQBUF, &buf) < 0) {
      printf("ioctl() VIDIOC_DQBUF failed.\n");
      break;
    }

    assert(buf.index < vid->n_buffers);

    vid->seq++;

    if(vid->trigger) {

      // todo add timestamp again
      //vid->img->timestamp = util_timestamp();
      vid->img->seq = vid->seq;
      memcpy(vid->img->buf, vid->buffers[buf.index].buf, vid->w*vid->h*2);
      vid->trigger=0;
    }

    if (ioctl(vid->fd, VIDIOC_QBUF, &buf) < 0) {
      printf("ioctl() VIDIOC_QBUF failed.\n");
      break;
    }
  }
  return 0;
}

int video_init(struct vid_struct *vid)
{
  struct v4l2_capability cap;
  struct v4l2_format fmt;
  unsigned int i;
  enum v4l2_buf_type type;

  vid->seq=0;
  vid->trigger=0;
  if(vid->n_buffers==0) vid->n_buffers=4;

  vid->fd = open(vid->device, O_RDWR | O_NONBLOCK, 0);

  if (ioctl(vid->fd, VIDIOC_QUERYCAP, &cap) < 0) {
    printf("ioctl() VIDIOC_QUERYCAP failed.\n");
    return -1;
  }

  //printf("2 driver = %s, card = %s, version = %d, capabilities = 0x%x\n", cap.driver, cap.card, cap.version, cap.capabilities);

  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = vid->w;
  fmt.fmt.pix.height = vid->h;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;

  if (ioctl(vid->fd, VIDIOC_S_FMT, &fmt) < 0) {
    printf("ioctl() VIDIOC_S_FMT failed.\n");
    return -1;
  }

  //image_size = fmt.fmt.pix.width * fmt.fmt.pix.height *3/2;

  struct v4l2_requestbuffers req;

  CLEAR(req);
  req.count = vid->n_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(vid->fd, VIDIOC_REQBUFS, &req) < 0) {
    printf("ioctl() VIDIOC_REQBUFS failed.\n");
    return -1;
  }

  printf("Buffer count = %d\n", vid->n_buffers);

  vid->buffers = (struct buffer_struct*)calloc(vid->n_buffers, sizeof(struct buffer_struct));

  for (i = 0; i < vid->n_buffers; ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(vid->fd, VIDIOC_QUERYBUF, &buf) < 0) {
      printf("ioctl() VIDIOC_QUERYBUF failed.\n");
      return -1;
    }

    vid->buffers[i].length = buf.length;
    printf("buffer%d.length=%d\n",i,buf.length);
    vid->buffers[i].buf = mmap(NULL, buf.length, PROT_READ|PROT_WRITE, MAP_SHARED, vid->fd, buf.m.offset);

    if (MAP_FAILED == vid->buffers[i].buf) {
      printf ("mmap() failed.\n");
      return -1;
    }
  }

  for (i = 0; i < vid->n_buffers; ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(vid->fd, VIDIOC_QBUF, &buf) < 0) {
      printf("ioctl() VIDIOC_QBUF failed.\n");
      return -1;
    }
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(vid->fd, VIDIOC_STREAMON, &type)< 0) {
    printf("ioctl() VIDIOC_STREAMON failed.\n");
    return -1;
  }

  //start video thread
  int rc = pthread_create(&video_thread, NULL, video_thread_main, vid);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
    return 202;
  }

  return 0;
}

void video_close(struct vid_struct *vid)
{
  int i;
  for (i = 0; i < (int)vid->n_buffers; ++i) {
    if (-1 == munmap(vid->buffers[i].buf, vid->buffers[i].length)) printf("munmap() failed.\n");
  }
  close(vid->fd);
}

struct img_struct *video_create_image(struct vid_struct *vid)
{
  struct img_struct* img = (struct img_struct*)malloc(sizeof(struct img_struct));
  img->w=vid->w;
  img->h=vid->h;
  img->buf = (unsigned char*)malloc(vid->h*vid->w*2);
  return img;
}

pthread_mutex_t video_grab_mutex = PTHREAD_MUTEX_INITIALIZER;

void video_grab_image(struct vid_struct *vid, struct img_struct *img) {
  pthread_mutex_lock(&video_grab_mutex);
  vid->img = img;
  vid->trigger=1;
  //  while(vid->trigger) pthread_yield();
  while(vid->trigger) usleep(1);
  pthread_mutex_unlock(&video_grab_mutex);
}
