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

#ifdef SYS_TIME_LED
#include "led.h"
#endif


void sys_time_arch_init( void ) {

  sys_time.cpu_ticks_per_sec = 1e6;
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);

  struct sigaction sa;
  struct itimerval timer;

  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = &sys_tick_handler;
  sigaction(SIGALRM, &sa, NULL);

  // timer expires after sys_time.resolution sec
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = USEC_OF_SEC(sys_time.resolution);
  // and every SYS_TIME_RESOLUTION sec after that
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = USEC_OF_SEC(sys_time.resolution);

  setitimer(ITIMER_REAL, &timer, NULL);
}

void sys_tick_handler( int signum ) {

  sys_time.nb_tick++;
  sys_time.nb_sec_rem += sys_time.resolution_cpu_ticks;;
  if (sys_time.nb_sec_rem >= sys_time.cpu_ticks_per_sec) {
    sys_time.nb_sec_rem -= sys_time.cpu_ticks_per_sec;
    sys_time.nb_sec++;
#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif

  }
  for (unsigned int i=0; i<SYS_TIME_NB_TIMER; i++) {
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
