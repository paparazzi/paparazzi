/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include "sys_time.h"

volatile bool_t sys_time_period_elapsed;
uint32_t cpu_time_ticks;

void sys_time_init( void ) {
  /* Generate SysTick interrupt every PERIODIC_TASK_PERIOD AHS clk */
  if(SysTick_Config(PERIODIC_TASK_PERIOD-1)) {
    while(1);
  }
  /* Set SysTick handler */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
  sys_time_period_elapsed = FALSE;
  cpu_time_ticks = 0;
}

void sys_tick_irq_handler(void) {
  sys_time_period_elapsed = TRUE;
}


