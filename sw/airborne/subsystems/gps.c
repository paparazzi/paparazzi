/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

/** @file gps.c
 *  @brief Device independent GPS code
 *
 */

#include "subsystems/gps.h"

#include "led.h"

#define MSEC_PER_WEEK (1000*60*60*24*7)

struct GpsState gps;

struct GpsTimeSync gps_time_sync;

void gps_init(void) {
  gps.fix = GPS_FIX_NONE;
  gps.cacc = 0;
#ifdef GPS_LED
  LED_OFF(GPS_LED);
#endif
#ifdef GPS_TYPE_H
  gps_impl_init();
#endif
}

uint32_t gps_tow_from_sys_ticks(uint32_t sys_ticks)
{
  uint32_t clock_delta;
  uint32_t time_delta;
  uint32_t itow_now;

  if (sys_ticks < gps_time_sync.t0_ticks) {
    clock_delta = (0xFFFFFFFF - sys_ticks) + gps_time_sync.t0_ticks + 1;
  } else {
    clock_delta = sys_ticks - gps_time_sync.t0_ticks;
  }

  time_delta = msec_of_sys_time_ticks(clock_delta);

  itow_now = gps_time_sync.t0_tow + time_delta;
  if (itow_now > MSEC_PER_WEEK) {
    itow_now %= MSEC_PER_WEEK;
  }

  return itow_now;
}
