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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * Dummy C implementation for simulation
 * The V4L2 could also work in simulation, but must be adapted a bit.
 */

// Own header
#include "video_thread.h"
#include "cv.h"

// Initialize the video_thread structure with the defaults
struct video_thread_t video_thread = {
  .is_running = FALSE,
  .fps = 30,
  .take_shot = FALSE,
  .shot_number = 0
};

// All dummy functions
void video_thread_init(void) {}
void video_thread_periodic(void)
{
  struct image_t img;
  image_create(&img, 320, 240, IMAGE_YUV422);
  int i, j;
  uint8_t u, v;

#ifdef SMARTUAV_SIMULATOR
  SMARTUAV_IMPORT(&img);
#else
  if (video_thread.is_running) {
    u = 0;
    v = 255;
  } else {
    u = 255;
    v = 0;
  }
  uint8_t *p = (uint8_t *) img.buf;
  for (j = 0; j < img.h; j++) {
    for (i = 0; i < img.w; i += 2) {
      *p++ = u;
      *p++ = j;
      *p++ = v;
      *p++ = j;
    }
  }
  video_thread.is_running = ! video_thread.is_running;
#endif

  cv_run(&img);

  image_free(&img);
}

void video_thread_start(void) {}
void video_thread_stop(void) {}
void video_thread_take_shot(bool take __attribute__((unused))) {}
