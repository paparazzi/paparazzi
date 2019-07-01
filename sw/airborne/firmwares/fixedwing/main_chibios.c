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
#include "mcu.h"
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

#if USE_HARD_FAULT_RECOVERY
#include "subsystems/datalink/downlink.h"
#endif

/*
 * Default autopilot thread stack size
 */
#ifndef AP_THREAD_STACK_SIZE
#define AP_THREAD_STACK_SIZE 8192
#endif

/*
 * PPRZ/AP thread
 */
static void thd_ap(void *arg);
static THD_WORKING_AREA(wa_thd_ap, AP_THREAD_STACK_SIZE);
static thread_t *apThdPtr = NULL;

/*
 * Default FBW thread stack size
 */
#ifndef FBW_THREAD_STACK_SIZE
#define FBW_THREAD_STACK_SIZE 1024
#endif

/*
 * PPRZ/FBW thread
 */
static void thd_fbw(void *arg);
static THD_WORKING_AREA(wa_thd_fbw, FBW_THREAD_STACK_SIZE);
static thread_t *fbwThdPtr = NULL;

/**
 * Main function
 */
int main(void)
{
  // Init
  Fbw(init);
#if USE_HARD_FAULT_RECOVERY
  // if recovering from hard fault, don't call AP init, only FBW
  if (!recovering_from_hard_fault) {
#endif
    Ap(init);
#if USE_HARD_FAULT_RECOVERY
  } else {
    // but we still need downlink to be initialized
    downlink_init();
    modules_datalink_init();
  }
#endif

  chThdSleepMilliseconds(100);

  // Create threads
  fbwThdPtr = chThdCreateStatic(wa_thd_fbw, sizeof(wa_thd_fbw), NORMALPRIO, thd_fbw, NULL);
#if USE_HARD_FAULT_RECOVERY
  // if recovering from hard fault, don't start AP thread, only FBW
  if (!recovering_from_hard_fault) {
#endif
    apThdPtr = chThdCreateStatic(wa_thd_ap, sizeof(wa_thd_ap), NORMALPRIO, thd_ap, NULL);
#if USE_HARD_FAULT_RECOVERY
  }
#endif

  // Main loop, do nothing
  chThdSleep(TIME_INFINITE);
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
    // In tick mode, the minimum step is 1e6 / CH_CFG_ST_FREQUENCY
    // which means that whatever happens, if we do a sleep of this
    // time step, the next wakeup will be "aligned" and we won't see
    // jitter. The polling on event will also be as fast as possible
    // Be careful that in tick-less mode, it will be required to use
    // the chThdSleepUntil function with a correct computation of the
    // wakeup time, in particular roll-over should be check.
    chThdSleepMicroseconds(1000000 / CH_CFG_ST_FREQUENCY);
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
    // In tick mode, the minimum step is 1e6 / CH_CFG_ST_FREQUENCY
    // which means that whatever happens, if we do a sleep of this
    // time step, the next wakeup will be "aligned" and we won't see
    // jitter. The polling on event will also be as fast as possible
    // Be careful that in tick-less mode, it will be required to use
    // the chThdSleepUntil function with a correct computation of the
    // wakeup time, in particular roll-over should be check.
    chThdSleepMicroseconds(1000000 / CH_CFG_ST_FREQUENCY);
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

