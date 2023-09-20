/*
 * Copyright (C) 2013-2021 Gautier Hattenberger, Alexandre Bustico
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file main_chibios.c
 *
 * Program main function for ChibiOS inplementation
 *
 * Calls AP thread on single/dual MCU
 */

#include "mcu_periph/sys_time.h"
#include "mcu.h"
#include <ch.h>

#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_CFG_ST_FREQUENCY
#elif  CH_CFG_ST_FREQUENCY < (2 * PERIODIC_FREQUENCY) && SYS_TIME_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_CFG_ST_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
#endif

#if (defined AP) && (defined FBW)
#error "AP and FBW can't be defined at the same time"
#endif
#if (!defined AP) && (!defined FBW)
#error "AP or FBW should be defined"
#endif

#ifdef FBW
#include "main_fbw.h"
#define Call(f) main_fbw_ ## f
#endif

#ifdef AP
#include "main_ap.h"
#define Call(f) main_ap_ ## f
#endif

#include "generated/modules.h"

#ifndef THD_WORKING_AREA_MAIN
#define THD_WORKING_AREA_MAIN 8192
#endif

/*
 * PPRZ/AP thread
 */
static void thd_ap(void *arg);
THD_WORKING_AREA(wa_thd_ap, THD_WORKING_AREA_MAIN);
static thread_t *apThdPtr = NULL;

#if USE_HARD_FAULT_RECOVERY
#include "main_recovery.h"

#ifndef THD_WORKING_AREA_RECOVERY
#define THD_WORKING_AREA_RECOVERY 1024
#endif

static void thd_recovery(void *arg);
THD_WORKING_AREA(wa_thd_recovery, THD_WORKING_AREA_RECOVERY);
static thread_t *recoveryThdPtr = NULL;
#endif

/**
 * Main function
 */
int main(void)
{
  // Init

  // mcu modules init in all cases
  modules_mcu_init();

#if USE_HARD_FAULT_RECOVERY
  // if recovering from hard fault, don't call AP init, only FBW
  if (!recovering_from_hard_fault) {
#endif
    Call(init());
    chThdSleepMilliseconds(100);

    // Create threads
    apThdPtr = chThdCreateStatic(wa_thd_ap, sizeof(wa_thd_ap), NORMALPRIO, thd_ap, NULL);

#if USE_HARD_FAULT_RECOVERY
  } else {
    main_recovery_init();
    chThdSleepMilliseconds(100);

    // Create threads
    recoveryThdPtr = chThdCreateStatic(wa_thd_recovery, sizeof(wa_thd_recovery), NORMALPRIO, thd_recovery, NULL);
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
    systime_t t = chVTGetSystemTime();
    Call(periodic());
    Call(event());
    // The sleep time is computed to have a polling interval of
    // 1e6 / CH_CFG_ST_FREQUENCY. If time is passed, thanks to the
    // "Windowed" sleep function, the execution is not blocked until
    // a complet roll-over.
    chThdSleepUntilWindowed(t, t + TIME_US2I(1000000 / CH_CFG_ST_FREQUENCY));
    //chThdSleepUntilWindowed(t, t + TIME_US2I(10));
  }

  chThdExit(0);
}

#if USE_HARD_FAULT_RECOVERY
/*
 * PPRZ/RECOVERY thread
 *
 * Call PPRZ minimal AP after MCU hard fault
 */
static void thd_recovery(void *arg)
{
  (void) arg;
  chRegSetThreadName("RECOVERY");

  while (!chThdShouldTerminateX()) {
    systime_t t = chVTGetSystemTime();
    main_recovery_periodic();
    main_recovery_event();
    // The sleep time is computed to have a polling interval of
    // 1e6 / CH_CFG_ST_FREQUENCY. If time is passed, thanks to the
    // "Windowed" sleep function, the execution is not blocked until
    // a complet roll-over.
    chThdSleepUntilWindowed(t, t + TIME_US2I(1000000 / CH_CFG_ST_FREQUENCY));
    //chThdSleepUntilWindowed(t, t + TIME_US2I(10));
  }

  chThdExit(0);
}
#endif

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
#if USE_HARD_FAULT_RECOVERY
  if (recoveryThdPtr != NULL) {
    chThdTerminate(recoveryThdPtr);
    chThdWait(recoveryThdPtr);
    recoveryThdPtr = NULL;
  }
#endif
}

