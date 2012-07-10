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
 */

/**
 * @file arch/stm32/mcu_periph/sys_time_arch.c
 * @brief STM32 timing functions.
 *
 */

#include "mcu_periph/sys_time.h"

#include "libopencm3/stm32/systick.h"

#ifdef SYS_TIME_LED
#include "led.h"
#endif

void sys_time_arch_init( void ) {

  /* Generate SysTick interrupt every SYS_TIME_RESOLUTION_CPU_TICKS
   * The timer interrupt is activated on the transition from 1 to 0,
   * therefore it activates every n+1 clock ticks.
   */
  systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB);
  systick_set_reload(SYS_TIME_RESOLUTION_CPU_TICKS-1);

  systick_interrupt_enable();
  systick_counter_enable();
}


// FIXME : nb_tick rollover ???
//
// 97 days at 512hz
// 12 hours at 100khz
//
void sys_tick_handler(void) {

  sys_time.nb_tick++;
  sys_time.nb_sec_rem += SYS_TIME_RESOLUTION_CPU_TICKS;
  if (sys_time.nb_sec_rem >= CPU_TICKS_PER_SEC) {
    sys_time.nb_sec_rem -= CPU_TICKS_PER_SEC;
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
      if (sys_time.timer[i].cb) sys_time.timer[i].cb(i);
    }
  }
}
