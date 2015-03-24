/*
 *
 * Copyright (C) 2009-2013 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/linux/mcu_periph/sys_time_arch.h
 * linux timing functions
 */

#include "mcu_periph/sys_time.h"
#include <stdio.h>
#include <pthread.h>
#include <sys/timerfd.h>
#include <time.h>
#include "rt_priority.h"

#ifdef SYS_TIME_LED
#include "led.h"
#endif

#ifndef SYS_TIME_THREAD_PRIO
#define SYS_TIME_THREAD_PRIO 29
#endif

pthread_t sys_time_thread;
static struct timespec startup_time;

static void sys_tick_handler(void);
void *sys_time_thread_main(void *data);

#define NSEC_OF_SEC(sec) ((sec) * 1e9)

void *sys_time_thread_main(void *data)
{
  int fd;

  /* Create the timer */
  fd = timerfd_create(CLOCK_MONOTONIC, 0);
  if (fd == -1) {
    perror("Could not set up timer.");
    return NULL;
  }

  get_rt_prio(SYS_TIME_THREAD_PRIO);

  /* Make the timer periodic */
  struct itimerspec timer;
  /* timer expires after sys_time.resolution sec */
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_nsec = NSEC_OF_SEC(sys_time.resolution);
  /* and every SYS_TIME_RESOLUTION sec after that */
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_nsec = NSEC_OF_SEC(sys_time.resolution);

  if (timerfd_settime(fd, 0, &timer, NULL) == -1) {
    perror("Could not set up timer.");
    return NULL;
  }

  while (1) {
    unsigned long long missed;
    /* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	int r = read(fd, &missed, sizeof(missed));
    if (r == -1) {
      perror("Couldn't read timer!");
    }
    if (missed > 1) {
      fprintf(stderr, "Missed %lld timer events!\n", missed);
    }
    /* set current sys_time */
    sys_tick_handler();
  }
  return NULL;
}

void sys_time_arch_init(void)
{
  sys_time.cpu_ticks_per_sec = 1e6;
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);

  clock_gettime(CLOCK_MONOTONIC, &startup_time);

  int ret = pthread_create(&sys_time_thread, NULL, sys_time_thread_main, NULL);
  if (ret) {
    perror("Could not setup sys_time_thread");
    return;
  }
}

static void sys_tick_handler(void)
{
  /* get current time */
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  /* time difference to startup */
  time_t d_sec = now.tv_sec - startup_time.tv_sec;
  long d_nsec = now.tv_nsec - startup_time.tv_nsec;

  /* wrap if negative nanoseconds */
  if (d_nsec < 0) {
    d_sec -= 1;
    d_nsec += 1000000000L;
  }

#ifdef SYS_TIME_LED
  if (d_sec > sys_time.nb_sec) {
    LED_TOGGLE(SYS_TIME_LED);
  }
#endif

  sys_time.nb_sec = d_sec;
  sys_time.nb_sec_rem = cpu_ticks_of_nsec(d_nsec);
  sys_time.nb_tick = sys_time_ticks_of_sec(d_sec) + sys_time_ticks_of_usec(d_nsec / 1000);

  /* advance virtual timers */
  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
        sys_time.nb_tick >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = TRUE;
      /* call registered callbacks, WARNING: they will be executed in the sys_time thread! */
      if (sys_time.timer[i].cb) {
        sys_time.timer[i].cb(i);
      }
    }
  }
}
