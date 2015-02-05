/*
 * Copyright (C) 2015 The Paparazzi Community
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/cv/framerate.c
 *
 */

#include "std.h"
#include "framerate.h"

// Frame Rate (FPS)
#include <sys/time.h>

// local variables
volatile long timestamp;
struct timeval start_time;
struct timeval end_time;

#define USEC_PER_SEC 1000000L

static long time_elapsed(struct timeval *t1, struct timeval *t2)
{
  long sec, usec;
  sec = t2->tv_sec - t1->tv_sec;
  usec = t2->tv_usec - t1->tv_usec;
  if (usec < 0) {
    --sec;
    usec = usec + USEC_PER_SEC;
  }
  return sec * USEC_PER_SEC + usec;
}

static void start_timer(void)
{
  gettimeofday(&start_time, NULL);
}

static long end_timer(void)
{
  gettimeofday(&end_time, NULL);
  return time_elapsed(&start_time, &end_time);
}


void framerate_init(void) {
  // Frame Rate Initialization
  timestamp = 0;
  start_timer();
}

float framerate_run(void) {
  // FPS
  timestamp = end_timer();
  float framerate_FPS = (float) 1000000 / (float)timestamp;
  //  printf("dt = %d, FPS = %f\n",timestamp, FPS);
  start_timer();
  return framerate_FPS;
}
