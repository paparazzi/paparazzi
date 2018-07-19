/*
 * Copyright (C) 2015 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
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

/* ChibiOS includes */
#include "ch.h"

/* paparazzi includes */
#include "mcu.h"
#include "led.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/uart_arch.h"


/*
 * Thread Area Definitions
 */
#define CH_CFG_THREAD_AREA_MAIN_PERIODIC 128

/*
 * Thread Area Initialization
 */
static THD_WORKING_AREA(wa_thd_main_periodic_05, CH_CFG_THREAD_AREA_MAIN_PERIODIC);
static THD_WORKING_AREA(wa_thd_rx, CH_CFG_THREAD_AREA_MAIN_PERIODIC);

/*
 * Static Thread Definitions
 */
static __attribute__((noreturn)) void thd_main_periodic_05(void *arg);
static __attribute__((noreturn)) void thd_rx(void *arg);

/*
 * Test Thread
 *
 * Replaces main_periodic_05()
 *
 */
static __attribute__((noreturn)) void thd_main_periodic_05(void *arg)
{
  chRegSetThreadName("thd_blinker");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += TIME_MS2I(500);
#ifdef SYS_TIME_LED
      LED_TOGGLE(SYS_TIME_LED);
#endif
    chThdSleepUntil(time);
  }
}

/*
 * Serial RX thread
 */
__attribute__((noreturn)) void thd_rx(void *arg)
{
  chRegSetThreadName("rx_thread");
  (void) arg;

  uint8_t charbuf;
  while (TRUE) {
#ifdef LED_GREEN
      LED_TOGGLE(LED_GREEN);
#endif
    charbuf = uart_getch(&SERIAL_PORT);
    uart_transmit(&SERIAL_PORT, charbuf);
  }
}

int main(void)
{
  mcu_init();

  /*
   * Init threads
   */
  chThdCreateStatic(wa_thd_main_periodic_05, sizeof(wa_thd_main_periodic_05), NORMALPRIO, thd_main_periodic_05, NULL);
  chThdCreateStatic(wa_thd_rx, sizeof(wa_thd_rx), NORMALPRIO, thd_rx, NULL);


  while (1) {
    /* sleep for 1s */
    sys_time_ssleep(1);
    uart_transmit(&SERIAL_PORT, 'N');
    uart_transmit(&SERIAL_PORT, 'i');
    uart_transmit(&SERIAL_PORT, 'c');
    uart_transmit(&SERIAL_PORT, 'e');

    sys_time_msleep(500);
    uint8_t tx_switch[] = " work!\r\n";
    uart_transmit_buffer(&SERIAL_PORT, tx_switch, sizeof(tx_switch));
  }

  return 0;
}
