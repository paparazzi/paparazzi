/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    Posix/hal_lld.c
 * @brief   Posix HAL subsystem low level driver code.
 *
 * @addtogroup POSIX_HAL
 * @{
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static struct timeval nextcnt;
static struct timeval tick = {0, 1000000 / CH_FREQUENCY};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief Low level HAL driver initialization.
 */
void hal_lld_init(void) {

#if defined(__APPLE__)
  puts("ChibiOS/RT simulator (OS X)\n");
#else
  puts("ChibiOS/RT simulator (Linux)\n");
#endif
  gettimeofday(&nextcnt, NULL);
  timeradd(&nextcnt, &tick, &nextcnt);
}

/**
 * @brief Interrupt simulation.
 */
void ChkIntSources(void) {
  struct timeval tv;

#if HAL_USE_SERIAL
  if (sd_lld_interrupt_pending()) {
    dbg_check_lock();
    if (chSchIsPreemptionRequired())
      chSchDoReschedule();
    dbg_check_unlock();
    return;
  }
#endif

  gettimeofday(&tv, NULL);
  if (timercmp(&tv, &nextcnt, >=)) {
    timeradd(&nextcnt, &tick, &nextcnt);

    CH_IRQ_PROLOGUE();

    chSysLockFromIsr();
    chSysTimerHandlerI();
    chSysUnlockFromIsr();

    CH_IRQ_EPILOGUE();

    dbg_check_lock();
    if (chSchIsPreemptionRequired())
      chSchDoReschedule();
    dbg_check_unlock();
  }
}

/** @} */
