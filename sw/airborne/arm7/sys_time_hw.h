/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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
 *\brief ARM7 timing functions 
 *
 */

#ifndef SYS_TIME_HW_H
#define SYS_TIME_HW_H

#include "std.h"
#include "LPC21xx.h"
#include CONFIG
#include "led.h"

extern uint32_t cpu_time_ticks;
static uint32_t last_periodic_event;

void TIMER0_ISR ( void ) __attribute__((naked));

/* T0 prescaler */
//#define T0_PCLK_DIV     3
#define T0_PCLK_DIV     1

static inline void sys_time_init( void ) {
  /* setup Timer 0 to count forever  */
  /* reset & disable timer 0         */
  T0TCR = TCR_RESET;
  /* set the prescale divider        */
  T0PR = T0_PCLK_DIV - 1;
  /* disable match registers         */
  T0MCR = 0;
  /* disable compare registers       */
  T0CCR = 0;
  /* disable external match register */
  T0EMR = 0;                          
  /* enable timer 0                  */
  T0TCR = TCR_ENABLE;

  cpu_time_sec = 0;
  cpu_time_ticks = 0;

  /* select TIMER0 as IRQ    */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER0);
  /* enable TIMER0 interrupt */
  VICIntEnable = VIC_BIT(VIC_TIMER0); 
  /* on slot vic slot 1      */
  VICVectCntl1 = VIC_ENABLE | VIC_TIMER0;
  /* address of the ISR      */
  VICVectAddr1 = (uint32_t)TIMER0_ISR; 

}

#define SYS_TICS_OF_SEC(s)   (uint32_t)(s * PCLK / T0_PCLK_DIV + 0.5)
#define SYS_TICS_OF_USEC(us) SYS_TICS_OF_SEC((us) * 1e-6)
#define SYS_TICS_OF_NSEC(ns) SYS_TICS_OF_SEC((ns) * 1e-9)
#define SIGNED_SYS_TICS_OF_SEC(s) (int32_t)(s * PCLK / T0_PCLK_DIV + 0.5)
#define SIGNED_SYS_TICS_OF_USEC(us) SIGNED_SYS_TICS_OF_SEC((us) * 1e-6)

#define FIFTY_MS          SYS_TICS_OF_SEC( 50e-3 )
#define AVR_PERIOD_MS     SYS_TICS_OF_SEC( 16.666e-3 )
#ifndef PERIODIC_TASK_PERIOD
#define PERIODIC_TASK_PERIOD AVR_PERIOD_MS
#endif

#define TIME_TICKS_PER_SEC SYS_TICS_OF_SEC(1)

static inline bool_t sys_time_periodic( void ) {
  uint32_t now = T0TC;
  uint32_t dif = now - last_periodic_event;
  if ( dif >= PERIODIC_TASK_PERIOD) {
    last_periodic_event += PERIODIC_TASK_PERIOD;
    cpu_time_ticks += PERIODIC_TASK_PERIOD;
    if (cpu_time_ticks > TIME_TICKS_PER_SEC) {
      cpu_time_ticks -= TIME_TICKS_PER_SEC;
      cpu_time_sec++;
#ifdef TIME_LED
      LED_TOGGLE(TIME_LED)
#endif
      }
    return TRUE;
  }
  return FALSE;
}

#endif /* SYS_TIME_HW_H */
