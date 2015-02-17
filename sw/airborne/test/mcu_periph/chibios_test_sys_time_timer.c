/*
 * Copyright (C) 2012 The Paparazzi Team
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
#include "ch.h"
#include "hal.h"
#include "std.h"
#include "mcu.h"
#include "led.h"
#include "mcu_periph/sys_time.h"

#define FREQUENCY_02 1./0.2
#define FREQUENCY_03 1./0.3
#define FREQUENCY_05 1./0.5

/*
 * Thread Area Definitions
 */
#define CH_THREAD_AREA_MAIN_PERIODIC 128

/*
 * Thread Area Initialization
 */
static WORKING_AREA(wa_thd_main_periodic_02, CH_THREAD_AREA_MAIN_PERIODIC);
static WORKING_AREA(wa_thd_main_periodic_03, CH_THREAD_AREA_MAIN_PERIODIC);
static WORKING_AREA(wa_thd_main_periodic_05, CH_THREAD_AREA_MAIN_PERIODIC);

/*
 * Static Thread Definitions
 */
static __attribute__((noreturn)) msg_t thd_main_periodic_02(void *arg);
static __attribute__((noreturn)) msg_t thd_main_periodic_03(void *arg);
static __attribute__((noreturn)) msg_t thd_main_periodic_05(void *arg);

/**
 * Test Thread
 *
 * Replaces main_periodic_02()
 *
 */
static __attribute__((noreturn)) msg_t thd_main_periodic_02(void *arg)
{
  chRegSetThreadName("thd_main_periodic_02");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/FREQUENCY_02);
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
static __attribute__((noreturn)) msg_t thd_main_periodic_03(void *arg)
{
  chRegSetThreadName("thd_main_periodic_03");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/FREQUENCY_03);
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
static __attribute__((noreturn)) msg_t thd_main_periodic_05(void *arg)
{
  chRegSetThreadName("thd_main_periodic_05");
  (void) arg;
  systime_t time = chTimeNow();
  while (TRUE)
  {
    time += US2ST(1000000/FREQUENCY_05);
#ifdef LED_RED
      LED_TOGGLE(LED_RED);
#endif
    chThdSleepUntil(time);
  }
}


int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
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
