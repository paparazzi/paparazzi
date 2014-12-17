/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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
 *
 */

#include "fms_periodic.h"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <time.h>
#include <string.h>
#include <errno.h>

#include "fms_debug.h"

static void fms_periodic_run(void);
//static void fms_periodic_delete(void);

struct FmsPeriodic {
  pid_t ap_pid;
};


int fms_periodic_init(void(*periodic_handler)(int))
{

  pid_t my_pid = fork();
  if (my_pid == -1) {
    TRACE(TRACE_ERROR, "fms_periodic : unable to fork : %s (%d)\n", strerror(errno), errno);
    return -1;
  }
  /* child process                 */
  else if (my_pid == 0) {
    fms_periodic_run();
  }
  /* succesful fork parent process */

  /* install signal handler */
  struct sigaction my_sigaction = {.sa_handler = periodic_handler };
  if (sigaction(SIGUSR1, &my_sigaction, NULL)) {
    TRACE(TRACE_ERROR, "fms_periodic : unable to install signal handler : %s (%d)\n", strerror(errno), errno);
    return -1;
  }

  /* set main process priority */
  struct sched_param param;
  param.sched_priority = 49;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    TRACE(TRACE_ERROR, "fms_periodic : hs sched_setscheduler failed : %s (%d)\n", strerror(errno), errno);
  }

  return 0;
}

//static void fms_periodic_delete(void) {
//
//}


#define NS_PER_SEC         1000000000
#define PERIODIC_DT_NSEC  (NS_PER_SEC/(FMS_PERIODIC_FREQ))

static void fms_periodic_run(void)
{

  struct sched_param param;
  param.sched_priority = 95;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    TRACE(TRACE_ERROR, "fms_periodic : hs sched_setscheduler failed : %s (%d)\n", strerror(errno), errno);
  }

  pid_t father_pid = getppid();

  struct timespec periodic_next;
  clock_gettime(CLOCK_MONOTONIC, &periodic_next);

  while (1) {
    periodic_next.tv_nsec += PERIODIC_DT_NSEC;
    while (periodic_next.tv_nsec > NS_PER_SEC) {
      periodic_next.tv_nsec -= NS_PER_SEC;
      periodic_next.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &periodic_next, NULL);
    kill(father_pid, SIGUSR1);
  }

  _exit(EXIT_SUCCESS);

}
