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

/**
 * @file arch/stm32/mcu_periph/sys_time_arch.h
 * @ingroup stm32_arch
 *
 * STM32 timing functions.
 *
 */

#ifndef SYS_TIME_ARCH_H
#define SYS_TIME_ARCH_H

#include "mcu_periph/sys_time.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include "std.h"
#ifdef RTOS_IS_CHIBIOS
#include "chibios_stub.h"
#include "chconf.h"
#endif

/**
 * Get the time in microseconds since startup.
 * WARNING: overflows after 70min!
 * @return microseconds since startup as uint32_t
 */
static inline uint32_t get_sys_time_usec(void)
{
#ifdef RTOS_IS_CHIBIOS
  return (chibios_chTimeNow() * (1000000 / CH_FREQUENCY));
#else
  return sys_time.nb_sec * 1000000 +
         usec_of_cpu_ticks(sys_time.nb_sec_rem) +
         usec_of_cpu_ticks(systick_get_reload() - systick_get_value());
#endif
}

/**
 * Get the time in milliseconds since startup.
 * @return milliseconds since startup as uint32_t
 */
static inline uint32_t get_sys_time_msec(void)
{
#ifdef RTOS_IS_CHIBIOS
  return (chibios_chTimeNow() * (1000 / CH_FREQUENCY));
#else
  return sys_time.nb_sec * 1000 +
         msec_of_cpu_ticks(sys_time.nb_sec_rem) +
         msec_of_cpu_ticks(systick_get_reload() - systick_get_value());
#endif
}


/** Busy wait in microseconds.
 *
 *  max value is limited by the max number of cycle
 *  i.e 2^32 * usec_of_cpu_ticks(systick_get_reload())
 */
static inline void sys_time_usleep(uint32_t us)
{
#ifdef RTOS_IS_CHIBIOS
  chibios_chThdSleepMicroseconds(us);
#else
  // start time
  uint32_t start = systick_get_value();
  // max time of one full counter cycle (n + 1 ticks)
  uint32_t DT = usec_of_cpu_ticks(systick_get_reload() + 1);
  // number of cycles
  uint32_t n = us / DT;
  // remaining number of cpu ticks
  uint32_t rem = cpu_ticks_of_usec(us % DT);
  // end time depend on the current value of the counter
  uint32_t end;
  if (rem < start) {
    end = start - rem;
  } else {
    // one more count flag is required
    n++;
    end = systick_get_reload() - rem + start;
  }
  // count number of cycles (when counter reachs 0)
  while (n) {
    while (!systick_get_countflag());
    n--;
  }
  // wait remaining ticks
  while (systick_get_value() > end);
#endif
}

#endif /* SYS_TIME_ARCH_H */
