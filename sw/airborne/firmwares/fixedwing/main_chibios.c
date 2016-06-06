/*
 * Copyright (C) 2013-2015 Gautier Hattenberger, Alexandre Bustico
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
 * @file firmwares/fixedwing/main_chibios.c
 */

#include "mcu_periph/sys_time.h"
#include <ch.h>

#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_CFG_ST_FREQUENCY
#elif SYS_TIME_FREQUENCY != CH_CFG_ST_FREQUENCY
#error SYS_TIME_FREQUENCY should be equal to CH_CFG_ST_FREQUENCY
#elif  CH_CFG_ST_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_CFG_ST_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
#endif

#ifdef FBW
#include "firmwares/fixedwing/main_fbw.h"
#define Fbw(f) f ## _fbw()
#else
#define Fbw(f)
#endif

#ifdef AP
#include "firmwares/fixedwing/main_ap.h"
#define Ap(f) f ## _ap()
#else
#define Ap(f)
#endif


/*
 * PPRZ thread
 */
static void thd_pprz(void *arg);
static THD_WORKING_AREA(wa_thd_pprz, 8192);
thread_t *pprzThdPtr = NULL;

/**
 * Main function
 */
int main(void)
{
  // Init
  Fbw(init);
  Ap(init);

  chThdSleepMilliseconds(100);

  // Create threads
  pprzThdPtr = chThdCreateStatic(wa_thd_pprz, sizeof(wa_thd_pprz),
      NORMALPRIO, thd_pprz, NULL);

  // Main loop, do nothing
  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
  return 0;
}

/*
 * PPRZ thread
 *
 * Call PPRZ periodic and event functions
 */
static void thd_pprz(void *arg)
{
  /*
     To be compatible with rtos architecture, each of this 4 workers should
     be implemented in differents threads, each of them waiting for job to be done:
     periodic task should sleep, and event task should wait for event
     */
  (void) arg;
  chRegSetThreadName("pprz big loop");

  while (!chThdShouldTerminateX()) {
    Fbw(handle_periodic_tasks);
    Ap(handle_periodic_tasks);
    Fbw(event_task);
    Ap(event_task);
    chThdSleepMicroseconds(500);
  }

  chThdExit(0);

}
