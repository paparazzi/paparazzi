/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2009-2010 The Paparazzi Team
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

/*
 *\brief STM32 timing functions
 *
 */

#ifndef SYS_TIME_HW_H
#define SYS_TIME_HW_H

#include "mcu_periph/sys_time.h"

#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include "std.h"

#define InitSysTimePeriodic()

#define SYS_TIME_TICS_OF_SEC(s)        (uint32_t)((s) * AHB_CLK + 0.5)
#define SYS_TIME_SIGNED_TICS_OF_SEC(s)  (int32_t)((s) * AHB_CLK + 0.5)

#define SysTimeTimerStart(_t) { _t = sys_time.nb_tic; }
#define SysTimeTimer(_t) (sys_time.nb_tic - (_t)))
#define SysTimeTimerStop(_t) { _t = (sys_time.nb_tic - (_t)); }


/** Busy wait, in microseconds */
/* for now empty shell */
static inline void sys_time_usleep(uint32_t us) {

}

#endif /* SYS_TIME_HW_H */
