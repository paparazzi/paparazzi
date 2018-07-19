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

/*
 * Thread Area Definitions
 */
#define CH_CFG_THREAD_AREA_MAIN_PERIODIC 128

/*
 * Thread Area Initialization
 */
static THD_WORKING_AREA(wa_thd_main_periodic_02, CH_CFG_THREAD_AREA_MAIN_PERIODIC);
static THD_WORKING_AREA(wa_thd_main_periodic_03, CH_CFG_THREAD_AREA_MAIN_PERIODIC);
static THD_WORKING_AREA(wa_thd_main_periodic_05, CH_CFG_THREAD_AREA_MAIN_PERIODIC);

/*
 * Static Thread Definitions
 */
static __attribute__((noreturn)) void thd_main_periodic_02(void *arg);
static __attribute__((noreturn)) void thd_main_periodic_03(void *arg);
static __attribute__((noreturn)) void thd_main_periodic_05(void *arg);

/**
 * Test Thread
 *
 * Replaces main_periodic_02()
 *
 */
static __attribute__((noreturn)) void thd_main_periodic_02(void *arg)
{
  chRegSetThreadName("thd_main_periodic_02");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += TIME_MS2I(200);
#ifdef LED_GREEN
      LED_TOGGLE(LED_GREEN);
#endif
    chThdSleepUntil(time);
  }
}

/**
 * Test Thread
 *
 * Replaces main_periodic_03()
 *
 */
static __attribute__((noreturn)) void thd_main_periodic_03(void *arg)
{
  chRegSetThreadName("thd_main_periodic_03");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += TIME_MS2I(300);
#ifdef SYS_TIME_LED
      LED_TOGGLE(SYS_TIME_LED);
#endif
    chThdSleepUntil(time);
  }
}

/**
 * Test Thread
 *
 * Replaces main_periodic_05()
 *
 */
static __attribute__((noreturn)) void thd_main_periodic_05(void *arg)
{
  chRegSetThreadName("thd_main_periodic_05");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += TIME_MS2I(500);
#ifdef LED_RED
      LED_TOGGLE(LED_RED);
#endif
    chThdSleepUntil(time);
  }
}


int main(void) {

  /* Paparazzi initialization.
   * Calls ChibiOS system initializations internally:
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   * Paparazzi initialization
   */
  mcu_init();

  /*
   * Init threads
   */
  chThdCreateStatic(wa_thd_main_periodic_02, sizeof(wa_thd_main_periodic_02), NORMALPRIO, thd_main_periodic_02, NULL);
  chThdCreateStatic(wa_thd_main_periodic_03, sizeof(wa_thd_main_periodic_03), NORMALPRIO, thd_main_periodic_03, NULL);
  chThdCreateStatic(wa_thd_main_periodic_05, sizeof(wa_thd_main_periodic_05), NORMALPRIO, thd_main_periodic_05, NULL);


  while(TRUE) {
    chThdSleepMilliseconds(500);
  }

  return 0;
}
