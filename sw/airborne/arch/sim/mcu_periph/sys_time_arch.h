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
 * Simulator timing functions
 */

#ifndef SYS_TIME_ARCH_H
#define SYS_TIME_ARCH_H

#include "std.h"

extern void sys_tick_handler(void);

// simulate 1us cpu ticks
#define CPU_TICKS_OF_SEC(s)        (uint32_t)((s) * 1e6 + 0.5)
#define SIGNED_CPU_TICKS_OF_SEC(s)  (int32_t)((s) * 1e6 + 0.5)

#define SEC_OF_CPU_TICKS(t)  ((t) / 1e6)
#define MSEC_OF_CPU_TICKS(t) ((t) / 1e3)
#define USEC_OF_CPU_TICKS(t) (t)

#define SysTimeTimerStart(_t) { }
#define SysTimeTimer(_t) (_t)
#define SysTimeTimerStop(_t) { }


static inline void sys_time_usleep(uint32_t us __attribute__ ((unused))) {}

#endif /* SYS_TIME_ARCH_H */
