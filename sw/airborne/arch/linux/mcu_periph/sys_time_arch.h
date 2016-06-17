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
 *
 */

/** @file arch/linux/mcu_periph/sys_time_arch.h
 * linux timing functions
 */

#ifndef SYS_TIME_ARCH_H
#define SYS_TIME_ARCH_H

#include "std.h"
#include <unistd.h>

/**
 * Get the time in microseconds since startup.
 * WARNING: overflows after 71min34seconds!
 * @return current system time as uint32_t
 */
static inline uint32_t get_sys_time_usec(void)
{
  return sys_time.nb_sec * 1000000 +
         usec_of_cpu_ticks(sys_time.nb_sec_rem);
}

/**
 * Get the time in milliseconds since startup.
 * @return milliseconds since startup as uint32_t
 */
static inline uint32_t get_sys_time_msec(void)
{
  return sys_time.nb_sec * 1000 +
         msec_of_cpu_ticks(sys_time.nb_sec_rem);
}

static inline void sys_time_usleep(uint32_t us)
{
  usleep(us);
}

/** elapsed time in microsecs between two timespecs */
static inline unsigned int sys_time_elapsed_us(struct timespec *prev, struct timespec *now)
{
  time_t d_sec = now->tv_sec - prev->tv_sec;
  long d_nsec = now->tv_nsec - prev->tv_nsec;
  /* wrap if negative nanoseconds */
  if (d_nsec < 0) {
    d_sec -= 1;
    d_nsec += 1000000000L;
  }
  return d_sec * 1000000 + d_nsec / 1000;
}

#endif /* SYS_TIME_ARCH_H */
