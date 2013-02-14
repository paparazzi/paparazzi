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

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/cm3/systick.h>
#include "std.h"

extern void sys_tick_handler(void);

/*
 * sys tick timer is running with AHB_CLK / 8
 */
#define CPU_TICKS_OF_SEC(s)        (uint32_t)((s) * (AHB_CLK / 8) + 0.5)
#define SIGNED_CPU_TICKS_OF_SEC(s)  (int32_t)((s) * (AHB_CLK / 8) + 0.5)

#define SEC_OF_CPU_TICKS(t)  ((t) / (AHB_CLK / 8))
#define MSEC_OF_CPU_TICKS(t) ((t) / (AHB_CLK / 8000))
#define USEC_OF_CPU_TICKS(t) ((t) / (AHB_CLK / 8000000))

/**
 * Get the time in microseconds since startup.
 * WARNING: overflows after 70min!
 * @return current system time as uint32_t
 */
static inline uint32_t get_sys_time_usec(void) {
  return sys_time.nb_sec * 1000000 +
    USEC_OF_CPU_TICKS(sys_time.nb_sec_rem) +
    USEC_OF_CPU_TICKS(STK_LOAD - systick_get_value());
}

/* Generic timer macros */
#define SysTimeTimerStart(_t) { _t = get_sys_time_usec(); }
#define SysTimeTimer(_t) ( get_sys_time_usec() - (_t))
#define SysTimeTimerStop(_t) { _t = ( get_sys_time_usec() - (_t)); }

/** Busy wait in microseconds.
 * Limited to ((2^24)-1)/9000000 = 1.86s
 */
static inline void sys_time_usleep(uint32_t us) {
  uint32_t start = systick_get_value();
  uint32_t ticks = CPU_TICKS_OF_USEC(us);
  /* cortex systick counts backwards */
  int32_t d = start - ticks;
  uint32_t end = 0;
  /* check if it wraps around zero */
  if (d >= 0) {
    end = d;
    while (systick_get_value() > end);
  } else {
    /* wait to zero */
    while (systick_get_value() > 0);
    /* wrap to reload value, wait the rest */
    end = STK_LOAD + d;
    while (systick_get_value() > end);
  }
}

#endif /* SYS_TIME_ARCH_H */
