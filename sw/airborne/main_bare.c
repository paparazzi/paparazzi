/*
 * Copyright (C) 2017-2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file main_bare.c
 *
 * Program main function with baremetal inplementation (no RTOS)
 *
 * Calls AP on single/dual MCU
 * Limit polling when used with an OS (e.g. Linux)
 * None on SITL
 */

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

#include "mcu_periph/sys_time.h"

#define POLLING_PERIOD (500000/PERIODIC_FREQUENCY)

#ifndef SITL
int main(void)
{
  // mcu modules init in all cases
  modules_mcu_init();

  Call(init());

#if LIMIT_EVENT_POLLING
  /* Limit main loop frequency to 1kHz.
   * This is a kludge until we can better leverage threads and have real events.
   * Without this limit the event flags will constantly polled as fast as possible,
   * resulting on 100% cpu load on boards with an (RT)OS.
   * On bare metal boards this is not an issue, as you have nothing else running anyway.
   */
  uint32_t t_begin = 0;
  uint32_t t_diff = 0;
  while (1) {
    t_begin = get_sys_time_usec();

    Call(periodic());
    Call(event());

    /* sleep remaining time to limit to polling frequency */
    t_diff = get_sys_time_usec() - t_begin;
    if (t_diff < POLLING_PERIOD) {
      sys_time_usleep(POLLING_PERIOD - t_diff);
    }
  }
#else
  while (1) {
    Call(periodic());
    Call(event());
  }
#endif

  return 0;
}
#endif /* SITL */

