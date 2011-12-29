/*
 * Paparazzi $Id$
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

#include "mcu_periph/sys_time.h"

#include "stm32_vector_table.h"
#ifdef SYS_TIME_LED
#include "subsystems/led.h"
#endif

void sys_time_init( void ) {

  /* Generate SysTick interrupt every SYS_TIME_RESOLUTION AHB_CLK */
  (void)SysTick_Config(SYS_TIME_RESOLUTION);
  /* Set SysTick handler priority                                  */
  NVIC_SetPriority(SysTick_IRQn, 0x0);

  sys_time.nb_sec     = 0;
  sys_time.nb_sec_rem = 0;
  sys_time.nb_tic     = 0;

  for (unsigned int i=0; i<SYS_TIME_NB_TIMER; i++) {
    sys_time.timer[i].in_use     = FALSE;
    sys_time.timer[i].cb         = NULL;
    sys_time.timer[i].elapsed    = FALSE;
    sys_time.timer[i].end_time   = 0;
    sys_time.timer[i].duration   = 0;
  }

}


// FIXME : nb_tic rollover ???
//
// 97 days at 512hz
// 12 hours at 100khz
//
void sys_tick_irq_handler(void) {

  sys_time.nb_tic++;
  sys_time.nb_sec_rem += SYS_TIME_RESOLUTION;
  if (sys_time.nb_sec_rem >= SYS_TIME_TICS_PER_SEC) {
    sys_time.nb_sec_rem -= SYS_TIME_TICS_PER_SEC;
    sys_time.nb_sec++;
#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif
  }
  for (unsigned int i=0; i<SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
    sys_time.nb_tic >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = TRUE;
      if (sys_time.timer[i].cb) sys_time.timer[i].cb(i);
    }
  }
}
