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
 * @file arch/lpc21/mcu_periph/sys_time_arch.h
 * @ingroup lpc21_arch
 *
 * LPC21xx timing functions.
 */

#ifndef SYS_TIME_ARCH_H
#define SYS_TIME_ARCH_H

#include "mcu_periph/sys_time.h"

#include BOARD_CONFIG
#include "LPC21xx.h"
#include "armVIC.h"

#include "std.h"

void TIMER0_ISR(void) __attribute__((naked));

/* T0 prescaler, set T0_CLK to 15MHz, T0_CLK = PCLK / T0PCLK_DIV */
#if (PCLK == 15000000)
#define T0_PCLK_DIV     1

#elif (PCLK == 30000000)
#define T0_PCLK_DIV     2

#elif (PCLK == 60000000)
#define T0_PCLK_DIV     4

#else
#error unknown PCLK frequency
#endif

#ifndef TIMER0_VIC_SLOT
#define TIMER0_VIC_SLOT 1
#endif /* TIMER0_VIC_SLOT */

/**
 * Get the time in microseconds since startup.
 * WARNING: overflows after ??
 * @return microseconds since startup as uint32_t
 */
static inline uint32_t get_sys_time_usec(void)
{
  return usec_of_cpu_ticks(T0TC);
}

/**
 * Get the time in milliseconds since startup.
 * @return milliseconds since startup as uint32_t
 */
static inline uint32_t get_sys_time_msec(void)
{
  return msec_of_cpu_ticks(T0TC);
}


#define SysTickTimerStart(_t) { _t = T0TC; }
#define SysTickTimer(_t) ((uint32_t)(T0TC - _t))
#define SysTickTimerStop(_t) { _t = (T0TC - _t); }

/** Busy wait, in microseconds */
static inline void sys_time_usleep(uint32_t us)
{
  uint32_t start = T0TC;
  uint32_t ticks = cpu_ticks_of_usec(us);
  while ((uint32_t)(T0TC - start) < ticks);
}

#endif /* SYS_TIME_ARCH_H */
