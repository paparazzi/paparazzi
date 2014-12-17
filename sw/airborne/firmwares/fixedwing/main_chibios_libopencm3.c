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

/**
 * @file firmwares/fixedwing/main_chibios_libopencm3.c
 */

#include "firmwares/fixedwing/chibios-libopencm3/chibios_init.h"
#include "mcu_periph/sys_time.h"
#include "chibios_stub.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
#include "sdLog.h"

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


static int32_t pprz_thd(void *arg);
static bool_t sdlogOk ;
bool_t pprzReady = FALSE;

int main(void)
{
  // Init
  sys_time_init();

  // Init ChibiOS
  sdlogOk = chibios_init();

  // Init PPRZ
  Fbw(init);
  Ap(init);

  chibios_chThdSleepMilliseconds(100);

  launch_pprz_thd(&pprz_thd);
  pprzReady = TRUE;
  // Call PPRZ periodic and event functions
  while (TRUE) {
    chibios_chThdSleepMilliseconds(1000);
  }
  return 0;
}


static int32_t pprz_thd(void *arg)
{
  /*
     To be compatible with rtos architecture, each of this 4 workers should
     be implemented in differents threads, each of them waiting for job to be done:
     periodic task should sleep, and event task should wait for event
     */
  (void) arg;
  chibios_chRegSetThreadName("pprz big loop");

  while (!chThdShouldTerminate()) {
    Fbw(handle_periodic_tasks);
    Ap(handle_periodic_tasks);
    Fbw(event_task);
    Ap(event_task);
    chibios_chThdSleepMilliseconds(1);
  }

  return 0;
}
