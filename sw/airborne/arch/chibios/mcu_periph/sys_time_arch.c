/*
 * Copyright (C) 2013-15 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
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
 */
#include "mcu_periph/sys_time_arch.h"
#include BOARD_CONFIG
#include "ch.h"
#include "led.h"

/*
 * Extra defines for ChibiOS CPU monitoring
 */
uint32_t core_free_memory;
uint8_t thread_counter;
uint32_t cpu_counter;
uint32_t idle_counter;
uint8_t cpu_frequency;

/*
 * Mutex guard
 */
mutex_t mtx_sys_time;

/*
 * Sys_tick handler thread
 */
static void thd_sys_tick(void *arg);
static THD_WORKING_AREA(wa_thd_sys_tick, 128);
static void sys_tick_handler(void);

/**
 * Sys time init
 * - start systick thread
 * - set counter resolutions
 */
void sys_time_arch_init(void)
{
  core_free_memory = 0;
  thread_counter = 0;
  cpu_counter = 0;
  idle_counter = 0;
  cpu_frequency = 0;

  sys_time.cpu_ticks_per_sec = AHB_CLK;

  /* cpu ticks per desired sys_time timer step */
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);

  chMtxObjectInit(&mtx_sys_time);

  // Create thread (Normal priority)
  chThdCreateStatic(wa_thd_sys_tick, sizeof(wa_thd_sys_tick),
      NORMALPRIO, thd_sys_tick, NULL);

}

uint32_t get_sys_time_usec(void)
{
  return (uint32_t)(chVTGetSystemTime() / CH_CFG_ST_FREQUENCY * 1000000);
}

uint32_t get_sys_time_msec(void)
{
  return (uint32_t)(chVTGetSystemTime() / CH_CFG_ST_FREQUENCY * 1000);
}

float get_sys_time_float_arch(void) {
  return  ((float)chVTGetSystemTime())/CH_CFG_ST_FREQUENCY;
}

/**
 * sys_time_usleep(uint32_t us)
 * Use only for up to 2^32/CH_CFG_ST_FREQUENCY-1 usec
 * e.g. if CH_CFG_ST_FREQUENCY=10000 use max for 420000 us
 * or 420ms, otherwise overflow happens
 */
void sys_time_usleep(uint32_t us)
{
  chThdSleep(US2ST(us));
}

void sys_time_msleep(uint16_t ms)
{
  chThdSleep(MS2ST(ms));
}

void sys_time_ssleep(uint8_t s)
{
  chThdSleep(S2ST(s));
}

/**
 * Sys_tick thread
 */
static __attribute__((noreturn)) void thd_sys_tick(void *arg)
{
  (void) arg;
  chRegSetThreadName("sys_tick_handler");
  systime_t time = chVTGetSystemTime();

  while (TRUE) {
    time += US2ST(1000); // endtime set to 1000us to avoid shift in time
    sys_tick_handler();
    chThdSleepUntil(time);
  }
}

/**
 * Increment counters
 */
static void sys_tick_handler(void)
{
  // Mutex guard
  chMtxLock(&mtx_sys_time);

  // current time in sys_ticks
  sys_time.nb_tick = chVTGetSystemTime();
  uint32_t sec = sys_time.nb_tick / CH_CFG_ST_FREQUENCY;
  sys_time.nb_sec = sec;
  sys_time.nb_sec_rem = sys_time.nb_tick - sys_time_ticks_of_sec(sys_time.nb_sec);

  // Mutex guard
  chMtxUnlock(&mtx_sys_time);
}


