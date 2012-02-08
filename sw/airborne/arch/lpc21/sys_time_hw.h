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
#include "mcu_periph/sys_time.h"
#include "std.h"
#include "LPC21xx.h"
#include BOARD_CONFIG
#include "led.h"
#include "armVIC.h"

extern uint32_t cpu_time_ticks;
extern uint32_t last_periodic_event;

void TIMER0_ISR ( void ) __attribute__((naked));

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


extern uint32_t sys_time_chrono_start; /* T0TC ticks */
extern uint32_t sys_time_chrono; /* T0TC ticks,frequency: PCLK / T0PCLK_DIV */
/* A division by SYS_TICS_OF_USEC(1) to get microseconds is too expensive
   for time measurement: about 2us */

#define SysTimeChronoStart() { sys_time_chrono_start = T0TC; }
#define SysTimeChronoStop() { sys_time_chrono = (T0TC - sys_time_chrono_start); }
/** Usage example, disabling IRQ and scaling to us to send
    disableIRQ(); SysTimeChronoStart();
    <Code to measure>
    SysTimeChronoStop(); enableIRQ();
    sys_time_chrono /=SYS_TICS_OF_USEC(1);
    DOWNLINK_SEND_CHRONO(42, &sys_time_chrono);
**/
#define SysTimeChronoStartDisableIRQ() { disableIRQ(); SysTimeChronoStart(); }
#define SysTimeChronoStopEnableIRQAndSendUS(_tag) { \
  SysTimeChronoStop(); \
  enableIRQ(); \
  sys_time_chrono /=SYS_TICS_OF_USEC(1); \
  DOWNLINK_SEND_CHRONO(_tag, &sys_time_chrono); \
}

/* Generic timer macros */
#define SysTimeTimerStart(_t) { _t = T0TC; }
#define SysTimeTimer(_t) ((uint32_t)(T0TC - _t))
#define SysTimeTimerStop(_t) { _t = (T0TC - _t); }


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
  _VIC_CNTL(TIMER0_VIC_SLOT) = VIC_ENABLE | VIC_TIMER0;
  /* address of the ISR      */
  _VIC_ADDR(TIMER0_VIC_SLOT) = (uint32_t)TIMER0_ISR;

}

#define SYS_TICS_OF_SEC(s)       (uint32_t)(s * PCLK / T0_PCLK_DIV + 0.5)
#define SIGNED_SYS_TICS_OF_SEC(s) (int32_t)(s * PCLK / T0_PCLK_DIV + 0.5)


#define SEC_OF_SYS_TICS(st)  (st / PCLK * T0_PCLK_DIV)
#define MSEC_OF_SYS_TICS(st) (st / (PCLK/1000) * T0_PCLK_DIV)
#define USEC_OF_SYS_TICS(st) (st / (PCLK/1000000) * T0_PCLK_DIV)

#define GET_CUR_TIME_FLOAT() ((float)cpu_time_sec + SEC_OF_SYS_TICS((float)cpu_time_ticks))

#ifndef PERIODIC_TASK_PERIOD
#define PERIODIC_TASK_PERIOD AVR_PERIOD_MS
#endif


#define InitSysTimePeriodic() last_periodic_event = T0TC;

static inline bool_t sys_time_periodic( void ) {
  uint32_t now = T0TC;
  uint32_t dif = now - last_periodic_event;
  if ( dif >= PERIODIC_TASK_PERIOD) {
    last_periodic_event += PERIODIC_TASK_PERIOD;
    cpu_time_ticks += PERIODIC_TASK_PERIOD;
    if (cpu_time_ticks > TIME_TICKS_PER_SEC) {
      cpu_time_ticks -= TIME_TICKS_PER_SEC;
      cpu_time_sec++;
#ifdef SYS_TIME_LED
      LED_TOGGLE(SYS_TIME_LED)
#endif
    }
    return TRUE;
  }
  return FALSE;
}

/** Busy wait, in microseconds */
static inline void sys_time_usleep(uint32_t us) {
  uint32_t start = T0TC;
  uint32_t ticks = SYS_TICS_OF_USEC(us);
  while ((uint32_t)(T0TC-start) < ticks);
}

#endif /* SYS_TIME_HW_H */
