/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file rt_priority.h
 * Functions to obtain rt priority or set the nice level.
 */

#ifndef RT_PRIORITY_H
#define RT_PRIORITY_H

#include <pthread.h>
#include <stdio.h>

static inline int get_rt_prio(int prio)
{
  struct sched_param param;
  int policy;
  pthread_getschedparam(pthread_self(), &policy, &param);
  printf("Current schedparam: policy %d, prio %d\n", policy, param.sched_priority);

  //SCHED_RR, SCHED_FIFO, SCHED_OTHER (POSIX scheduling policies)
  int sched = SCHED_FIFO;
  int min = sched_get_priority_min(sched);
  int max = sched_get_priority_max(sched);
  param.sched_priority = prio;
  if (prio > max || prio < min) {
    printf("Requested prio %d outside current min/max prios: %d/%d\n", prio, min, max);
    if (prio > max) {
      param.sched_priority = max;
    } else {
      param.sched_priority = min;
    }
  }

  if (pthread_setschedparam(pthread_self(), sched, &param)) {
    perror("setschedparam failed!");
    return -1;
  }
  else {
    pthread_getschedparam(pthread_self(), &policy, &param);
    printf("New schedparam: policy %d, prio %d\n", policy, param.sched_priority);
  }

  return 0;
}

#include <sys/resource.h>
#include <unistd.h>
#include <sys/syscall.h>

static inline int set_nice_level(int level)
{
  pid_t tid;
  tid = syscall(SYS_gettid);

  return setpriority(PRIO_PROCESS, tid, level);
}

#endif /* RT_PRIORITY_H */
