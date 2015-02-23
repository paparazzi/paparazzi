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
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/timerfd.h>

#ifdef SYS_TIME_LED
#include "led.h"
#endif

pthread_t sys_time_thread;
static void sys_tick_handler(void);
void *sys_time_thread_main(void *data);

#define NSEC_OF_SEC(sec) ((sec) * 1e9)

static inline int get_rt_prio(int prio)
{
  struct sched_param param;
  int policy;
  pthread_getschedparam(pthread_self(), &policy, &param);
  printf("Current shedparam: policy %d, prio %d\n", policy, param.sched_priority);

  //SCHED_RR, SCHED_FIFO, SCHED_OTHER (POSIX scheduling policies)
  int sched = SCHED_FIFO;
  int min = sched_get_priority_min(sched);
  int max = sched_get_priority_max(sched);
  printf("Current min/max prios: %d/%d\n", min, max);
  param.sched_priority = prio;
  if (pthread_setschedparam(pthread_self(), sched, &param)) {
    perror("setchedparam failed!");
    return -1;
  }
  else {
    pthread_getschedparam(pthread_self(), &policy, &param);
    printf("New shedparam: policy %d, prio %d\n", policy, param.sched_priority);
  }

  return 0;
}

void *sys_time_thread_main(void *data)
{
  int fd;

  /* Create the timer */
  fd = timerfd_create(CLOCK_MONOTONIC, 0);
  if (fd == -1) {
    perror("Could not set up timer.");
    return NULL;
  }

  get_rt_prio(29);

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
      printf("Missed %lld timer events!\n", missed);
    }
    /* advance sys_time */
    sys_tick_handler();
  }
  return NULL;
}

void sys_time_arch_init(void)
{

  sys_time.cpu_ticks_per_sec = 1e6;
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);


  int ret = pthread_create(&sys_time_thread, NULL, sys_time_thread_main, NULL);
  if (ret) {
    perror("Could not setup sys_time_thread");
    return;
  }

}

static void sys_tick_handler(void)
{

  sys_time.nb_tick++;
  sys_time.nb_sec_rem += sys_time.resolution_cpu_ticks;;
  if (sys_time.nb_sec_rem >= sys_time.cpu_ticks_per_sec) {
    sys_time.nb_sec_rem -= sys_time.cpu_ticks_per_sec;
    sys_time.nb_sec++;
#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif

  }
  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
        sys_time.nb_tick >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = TRUE;
      if (sys_time.timer[i].cb) {
        sys_time.timer[i].cb(i);
      }
    }
  }
}
