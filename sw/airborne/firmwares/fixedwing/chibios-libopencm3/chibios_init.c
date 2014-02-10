/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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

/*
 * @file firmwares/fixedwing/chibios-libopencm3/chibios_init.c
 *
 */

#include <ch.h>
#include <hal.h>
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
#include "sdLog.h"
#include "usbStorage.h"
#include "pprz_stub.h"
#include "rtcAccess.h"
#include "airframe.h"

// Delay before starting SD log
#ifndef SDLOG_START_DELAY
#define SDLOG_START_DELAY 30
#endif


#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_FREQUENCY
#elif SYS_TIME_FREQUENCY != CH_FREQUENCY
#error SYS_TIME_FREQUENCY should be equal to CH_FREQUENCY
#elif  CH_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
#endif

static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);
#define MAX(x , y)  (((x) > (y)) ? (x) : (y))
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

static WORKING_AREA(wa_thd_heartbeat, 2048);
void chibios_launch_heartbeat (void);
bool_t sdOk = FALSE;

#if LOG_PROCESS_STATE
static int32_t get_stack_free (Thread *tp);
#endif

/*
 * Init ChibiOS HAL and Sys
 */
bool_t chibios_init(void) {
  halInit();
  chSysInit();

  PWR->CSR &= ~PWR_CSR_BRE;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat),
      NORMALPRIO, thd_heartbeat, NULL);

  usbStorageStartPolling ();
  return RDY_OK;
}

static WORKING_AREA(pprzThd, 4096);
void launch_pprz_thd (int32_t (*thd) (void *arg))
{
  chThdCreateStatic(pprzThd, sizeof(pprzThd), NORMALPRIO+1, thd, NULL);
}


/*
 * Heartbeat thread
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg)
{
  (void) arg;
  chRegSetThreadName("pprz heartbeat");

  chThdSleepSeconds (SDLOG_START_DELAY);
  if (usbStorageIsItRunning ()) 
    chThdSleepSeconds (20000); // stuck here for hours
  else 
    sdOk = chibios_logInit(true);

  while (TRUE) {
    palTogglePad (GPIOC, GPIOC_LED3);
    chThdSleepMilliseconds (sdOk == TRUE ? 1000 : 200);
    static uint32_t timestamp = 0;

#if LOG_PROCESS_STATE
    sdLogWriteLog (&processLogFile, "    addr    stack  frestk prio refs  state  time name\r\n");
#endif

    // chSysDisable ();
    Thread *tp = chRegFirstThread();
    do {

#if LOG_PROCESS_STATE
      sdLogWriteLog (&processLogFile, "%.8lx %.8lx %6lu %4lu %4lu [S:%d] %5lu %s\r\n",
          (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
          get_stack_free (tp),
          (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
          tp->p_state, (uint32_t)tp->p_time,
          chRegGetThreadName(tp));
#endif

      tp = chRegNextThread(tp);
    } while (tp != NULL);
    // chSysEnable ();

    // we sync gps time to rtc every 5 seconds
    if (chTimeNow() - timestamp > 5000) {
      timestamp = chTimeNow();
      if (getGpsTimeOfWeek() != 0) {
        setRtcFromGps (getGpsWeek(), getGpsTimeOfWeek());
      }
    }

  }
}


#if LOG_PROCESS_STATE
static int32_t get_stack_free (Thread *tp)
{
  int32_t index = 0;
  const uint8_t *maxRamAddr =  (uint8_t*) (0x20000000 + (128*1024));
  const int32_t internalStructSize = 80;

  unsigned long long *stkAdr =  (unsigned long long *) ((uint8_t *) tp  + internalStructSize);
  //unsigned long long *stkAdr =  (unsigned long long *) tp;

  while ((stkAdr[index] == 0x5555555555555555) && ( ((uint8_t *) &(stkAdr[index])) < maxRamAddr))
    index++;

  const int32_t freeBytes = index * sizeof(long long);
  return MAX(0, freeBytes - internalStructSize);
}
#endif
