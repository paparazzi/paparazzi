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
 * PPRZ/AP thread
 */
static void thd_ap(void *arg);
static THD_WORKING_AREA(wa_thd_ap, 8192);
static thread_t *apThdPtr = NULL;

/*
 * PPRZ/FBW thread
 */
static void thd_fbw(void *arg);
static THD_WORKING_AREA(wa_thd_fbw, 1024);
static thread_t *fbwThdPtr = NULL;

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
  apThdPtr = chThdCreateStatic(wa_thd_ap, sizeof(wa_thd_ap), NORMALPRIO, thd_ap, NULL);
  fbwThdPtr = chThdCreateStatic(wa_thd_fbw, sizeof(wa_thd_fbw), NORMALPRIO, thd_fbw, NULL);

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
    Ap(handle_periodic_tasks);
    Ap(event_task);
    chThdSleepMicroseconds(500);
  }

  chThdExit(0);
}

/*
 * PPRZ/FBW thread
 *
 * Call PPRZ FBW periodic and event functions
 */
static void thd_fbw(void *arg)
{
  (void) arg;
  chRegSetThreadName("FBW");

  while (!chThdShouldTerminateX()) {
    Fbw(handle_periodic_tasks);
    Fbw(event_task);
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
  if (fbwThdPtr != NULL) {
    chThdTerminate(fbwThdPtr);
    chThdWait(fbwThdPtr);
    fbwThdPtr = NULL;
  }
}

