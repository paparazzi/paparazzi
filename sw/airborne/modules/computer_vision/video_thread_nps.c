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

// Initialize the video_thread structure with the defaults
struct video_thread_t video_thread = {
  .is_running = FALSE,
  .fps = 30,
  .take_shot = FALSE,
  .shot_number = 0
};

// All dummy functions
void video_thread_init(void) {}
void video_thread_periodic(void) {}
void video_thread_start(void) {}
void video_thread_stop(void) {}
void video_thread_take_shot(bool_t take __attribute__((unused))) {}
