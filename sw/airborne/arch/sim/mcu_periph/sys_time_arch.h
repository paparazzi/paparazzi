/*
 *
 * Copyright (C) 2009-2011 The Paparazzi Team
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

/** @file arch/sim/mcu_periph/sys_time_arch.h
 * Simulator timing functions.
 */

#ifndef SYS_TIME_ARCH_H
#define SYS_TIME_ARCH_H

#include "std.h"

extern void sys_tick_handler(void);

/**
 * Get the time in microseconds since startup.
 * @return microseconds since startup as uint32_t
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


static inline void sys_time_usleep(uint32_t us __attribute__((unused))) {}

#endif /* SYS_TIME_ARCH_H */
