/*
    video.h - video driver

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

#ifndef _VIDEO_H
#define _VIDEO_H

#include "../../cv/image.h"

struct buffer_struct {
  void * buf;
  size_t length;
};

struct vid_struct {
  char *device;
  int w;
  int h;
  int seq;
  unsigned int n_buffers;

  //private members
  int trigger;
  struct img_struct *img;
  struct buffer_struct * buffers;
  int fd;
};


int video_init(struct vid_struct *vid);
struct img_struct *video_create_image(struct vid_struct *vid);

void video_grab_image(struct vid_struct *vid, struct img_struct *img);
void video_close(struct vid_struct *vid);

#endif
