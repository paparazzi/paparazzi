/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
 * Copyright (C) 2015 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file arch/chibios/mcu_periph/sys_time_arch.c
 * Implementation of system time functions for ChibiOS arch
 *
 * Mostly empty functions for Paparazzi compatibility,
 * since ChibiOS uses different system time functions.
 */
#include "mcu_periph/sys_time_arch.h"
#include BOARD_CONFIG
#include <ch.h>
#include "led.h"

static MUTEX_DECL(sys_time_mtx);

/*
 * Sys_tick handler thread
 */
static void thd_sys_tick(void *arg);
static THD_WORKING_AREA(wa_thd_sys_tick, 1024);
static void sys_tick_handler(void);

static uint32_t cpu_ticks = 0;
static uint32_t cpu_counter = 0;

void sys_time_arch_init(void)
{

  sys_time.cpu_ticks_per_sec = CH_CFG_ST_FREQUENCY;

  /* cpu ticks per desired sys_time timer step */
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);

  // Create thread (PRIO should be higher than AP threads
  chThdCreateStatic(wa_thd_sys_tick, sizeof(wa_thd_sys_tick),
                    NORMALPRIO + 2, thd_sys_tick, NULL);

}

/**
 * Get the time in microseconds since startup.
 * WARNING: overflows after 70min!
 * @return microseconds since startup as uint32_t
 */
uint32_t get_sys_time_usec(void)
{
  chMtxLock(&sys_time_mtx);
  uint32_t current = chSysGetRealtimeCounterX();
  uint32_t t = sys_time.nb_sec * 1000000 +
               TIME_I2US(sys_time.nb_sec_rem) +
               RTC2US(STM32_SYSCLK, current - cpu_counter);
  chMtxUnlock(&sys_time_mtx);
  return t;
}

uint32_t get_sys_time_msec(void)
{
  chMtxLock(&sys_time_mtx);
  uint32_t current = chSysGetRealtimeCounterX();
  uint32_t t = sys_time.nb_sec * 1000 +
               TIME_I2MS(sys_time.nb_sec_rem) +
               RTC2MS(STM32_SYSCLK, current - cpu_counter);
  chMtxUnlock(&sys_time_mtx);
  return t;
}

/**
 * sys_time_usleep(uint32_t us)
 *
 * using intermediate 64 bits variable to avoid wrapping
 *
 * max sleep time is around 10 days (2^32 / CH_CFG_ST_FREQUENCY) at 10kHz
 */
void sys_time_usleep(uint32_t us)
{
  if (us < 1000) {
    // for small time, use the polled version instead of thread sleep
    chSysDisable();
    chSysPolledDelayX(US2RTC(STM32_HCLK, us));
    chSysEnable();
  } else {
    chThdSleepMicroseconds(us);
  }
}

void sys_time_msleep(uint16_t ms)
{
  chThdSleepMilliseconds(ms);
}

void sys_time_ssleep(uint8_t s)
{
  chThdSleepSeconds(s);
}


/*
 * Sys_tick thread
 */
static __attribute__((noreturn)) void thd_sys_tick(void *arg)
{
  (void) arg;
  chRegSetThreadName("sys_tick_handler");

  while (TRUE) {
    systime_t t = chVTGetSystemTime();
    sys_tick_handler();
    chThdSleepUntilWindowed(t, t + TIME_US2I(USEC_OF_SEC(1.f / (SYS_TIME_FREQUENCY))));
  }
}

static void sys_tick_handler(void)
{
  chMtxLock(&sys_time_mtx);
  /* current time in sys_ticks */
  sys_time.nb_tick++;
  /* max time is 2^32 / CH_CFG_ST_FREQUENCY, i.e. around 10 days at 10kHz */
  cpu_ticks = chVTGetSystemTime();
  /* store high resolution counter for accurate timing */
  cpu_counter = chSysGetRealtimeCounterX();
  /* increment secondes and remainder */
  uint32_t sec = cpu_ticks / CH_CFG_ST_FREQUENCY;
#ifdef SYS_TIME_LED
  if (sec > sys_time.nb_sec) {
    LED_TOGGLE(SYS_TIME_LED);
  }
#endif
  sys_time.nb_sec = sec;
  sys_time.nb_sec_rem = cpu_ticks - cpu_ticks_of_sec(sys_time.nb_sec); // in CPU ticks

  /* advance virtual timers */
  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
        sys_time.nb_tick >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = TRUE;
      /* call registered callbacks, WARNING: they will be executed in the sys_time thread! */
      if (sys_time.timer[i].cb) {
        sys_time.timer[i].cb(i);
      }
    }
  }
  chMtxUnlock(&sys_time_mtx);
}

