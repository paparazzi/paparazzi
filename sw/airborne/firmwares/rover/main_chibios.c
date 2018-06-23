/*
 * Copyright (C) 2018 Gautier Hattenberger, Alexandre Bustico
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rover/main_chibios.c
 */

#include "mcu_periph/sys_time.h"
#include "mcu.h"
#include <ch.h>

#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_CFG_ST_FREQUENCY
#elif SYS_TIME_FREQUENCY != CH_CFG_ST_FREQUENCY
#error SYS_TIME_FREQUENCY should be equal to CH_CFG_ST_FREQUENCY
#elif  CH_CFG_ST_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_CFG_ST_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
#endif

#include "firmwares/rotorcraft/main_ap.h"

#ifndef THD_WORKING_AREA_MAIN
#define THD_WORKING_AREA_MAIN 8192
#endif

/*
 * PPRZ/AP thread
 * Also include FBW on single MCU
 */
static void thd_ap(void *arg);
THD_WORKING_AREA(wa_thd_ap, THD_WORKING_AREA_MAIN);
static thread_t *apThdPtr = NULL;

/**
 * Main function
 */
int main(void)
{
  // Init
  main_init();

  chThdSleepMilliseconds(100);

  // Create threads
  apThdPtr = chThdCreateStatic(wa_thd_ap, sizeof(wa_thd_ap), NORMALPRIO, thd_ap, NULL);

  // Main loop, do nothing
  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
  return 0;
}

/*
 * PPRZ/AP thread
 *
 * Call PPRZ AP periodic and event functions
 */
static void thd_ap(void *arg)
{
  (void) arg;
  chRegSetThreadName("AP");

  while (!chThdShouldTerminateX()) {
    handle_periodic_tasks();
    main_event();
    chThdSleepMicroseconds(500);
  }

  chThdExit(0);
}

/*
 * Terminate autopilot threads
 * Wait until proper stop
 */
void pprz_terminate_autopilot_threads(void)
{
  if (apThdPtr != NULL) {
    chThdTerminate(apThdPtr);
    chThdWait(apThdPtr);
    apThdPtr = NULL;
  }
}

