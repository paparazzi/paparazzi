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

//#include "firmwares/fixedwing/chibios-libopencm3/chibios_init.h"
//#include "chibios_stub.h"
//#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
//#include "sdLog.h"

#include "mcu_periph/sys_time.h"

#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_FREQUENCY
#elif SYS_TIME_FREQUENCY != CH_FREQUENCY
#error SYS_TIME_FREQUENCY should be equal to CH_FREQUENCY
#elif  CH_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
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

#include "led.h"

/*
 * Heartbeat thread
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg);
static WORKING_AREA(wa_thd_heartbeat, 2048);

/*
 * PPRZ thread
 */
static msg_t thd_pprz(void *arg);
static WORKING_AREA(wa_thd_pprz, 4096);
Thread *pprzThdPtr = NULL;

/**
 * Main function
 */
int main(void)
{
  // Init
  Fbw(init);
  Ap(init);

  // ????
  PWR->CSR &= ~PWR_CSR_BRE;
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

  // Create threads
  chThdCreateStatic(wa_thd_heartbeat, sizeof(wa_thd_heartbeat),
      NORMALPRIO, thd_heartbeat, NULL);

  chThdSleepMilliseconds(100);

  pprzThdPtr = chThdCreateStatic(wa_thd_pprz, sizeof(wa_thd_pprz),
      NORMALPRIO+1, thd_pprz, NULL);

  // Main loop, do nothing
  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
  return 0;
}


/*
 * Heartbeat thread
 */
static __attribute__((noreturn)) msg_t thd_heartbeat(void *arg)
{
  (void) arg;
  chRegSetThreadName("pprz heartbeat");

  //chThdSleepSeconds (SDLOG_START_DELAY);
  //if (usbStorageIsItRunning ())
  //  chThdSleepSeconds (20000); // stuck here for hours
  //else
  //  sdOk = chibios_logInit();

  while (TRUE) {
    //palTogglePad (GPIOC, GPIOC_LED3);
    //chThdSleepMilliseconds (sdOk == TRUE ? 1000 : 200);
    //static uint32_t timestamp = 0;

    Thread *tp = chRegFirstThread();
    do {
      tp = chRegNextThread(tp);
    } while (tp != NULL);
    // we sync gps time to rtc every 5 seconds
    //if (chTimeNow() - timestamp > 5000) {
    //  timestamp = chTimeNow();
    //  if (getGpsTimeOfWeek() != 0) {
    //    setRtcFromGps (getGpsWeek(), getGpsTimeOfWeek());
    //  }
    //}

  }
}

/*
 * PPRZ thread
 *
 * Call PPRZ periodic and event functions
 */
static msg_t thd_pprz(void *arg)
{
  /*
     To be compatible with rtos architecture, each of this 4 workers should
     be implemented in differents threads, each of them waiting for job to be done:
     periodic task should sleep, and event task should wait for event
     */
  (void) arg;
  chRegSetThreadName("pprz big loop");

  while (!chThdShouldTerminate()) {
    Fbw(handle_periodic_tasks);
    Ap(handle_periodic_tasks);
    Fbw(event_task);
    Ap(event_task);
    chThdSleepMilliseconds(1);
  }

  return 0;
}
