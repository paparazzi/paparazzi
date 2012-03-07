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
 * @brief STM32 timing functions.
 *
 */

#ifndef SYS_TIME_ARCH_H
#define SYS_TIME_ARCH_H

#include "mcu_periph/sys_time.h"

#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include "std.h"

extern void sys_tick_irq_handler(void);

#define CPU_TICKS_OF_SEC(s)        (uint32_t)((s) * AHB_CLK + 0.5)
#define SIGNED_CPU_TICKS_OF_SEC(s)  (int32_t)((s) * AHB_CLK + 0.5)

#define SEC_OF_CPU_TICKS(t)  ((t) / AHB_CLK)
#define MSEC_OF_CPU_TICKS(t) ((t) / (AHB_CLK/1000))
#define USEC_OF_CPU_TICKS(t) ((t) / (AHB_CLK/1000000))

#define GET_CUR_TIME_USEC() (sys_time.nb_sec * 1000000 +                \
                             USEC_OF_CPU_TICKS(sys_time.nb_sec_rem) +   \
                             USEC_OF_CPU_TICKS(SysTick->LOAD - SysTick->VAL))

#define SysTimeTimerStart(_t) { _t = GET_CUR_TIME_USEC(); }
#define SysTimeTimer(_t) ( GET_CUR_TIME_USEC() - (_t))
#define SysTimeTimerStop(_t) { _t = ( GET_CUR_TIME_USEC() - (_t)); }


/** Busy wait, in microseconds */
// FIXME: directly use the SysTick->VAL here
static inline void sys_time_usleep(uint32_t us) {
  uint32_t end = GET_CUR_TIME_USEC() + us;
  while ((uint32_t)GET_CUR_TIME_USEC() < end);
}

#endif /* SYS_TIME_ARCH_H */
