/*
 * Copyright (C) 2010  Gautier Hattenberger
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
 *
 */
/** \file sys_mon.c
 *
 * System monitoring for bare metal targets
 * return cpu load, average exec time, ...
 */

#include "core/sys_mon.h"
#include "core/sys_mon_bare_metal.h"
#include "mcu_periph/sys_time.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

/** Global system monitor data (averaged over 1 sec) */
struct SysMon sys_mon;

/* Local vars */
static uint16_t n_periodic;
static uint16_t n_event;
static uint32_t periodic_timer;
static uint32_t periodic_cycle;
static uint32_t event_timer;
static uint32_t sum_time_periodic;  ///< in usec
static uint32_t sum_cycle_periodic; ///< in usec
static uint32_t sum_time_event;     ///< in usec
static uint32_t min_time_event;     ///< in usec
static uint32_t sum_n_event;

void init_sysmon(void)
{
  sys_mon.cpu_load = 0;
  sys_mon.periodic_time = 0;
  sys_mon.periodic_time_min = 0xFFFF;
  sys_mon.periodic_time_max = 0;
  sys_mon.periodic_cycle = 0;
  sys_mon.periodic_cycle_min = 0xFFFF;
  sys_mon.periodic_cycle_max = 0;
  sys_mon.event_number = 0;
  sys_mon.cpu_time = 0;

  n_periodic = 0;
  n_event = 0;
  sum_time_periodic = 0;
  sum_cycle_periodic = 0;
  sum_time_event = 0;
  min_time_event = ~0;
  sum_n_event = 0;
  periodic_timer = 0;
}

void periodic_report_sysmon(void)
{
  /** Report system status at low frequency */
  if (n_periodic > 0) {
    sys_mon.periodic_time = Max(sum_time_periodic / n_periodic, 1);
    sys_mon.periodic_cycle = sum_cycle_periodic / n_periodic;
    sys_mon.cpu_load = 100 * sys_mon.periodic_cycle / sys_mon.periodic_time;
    sys_mon.event_number = sum_n_event / n_periodic;
    sys_mon.cpu_time = get_sys_time_float();

    DOWNLINK_SEND_SYS_MON(DefaultChannel, DefaultDevice, &sys_mon.periodic_time,
                          &sys_mon.periodic_time_min, &sys_mon.periodic_time_max,
                          &sys_mon.periodic_cycle, &sys_mon.periodic_cycle_min,
                          &sys_mon.periodic_cycle_max, &sys_mon.event_number,
                          &sys_mon.cpu_load, &sys_mon.cpu_time);
  }

  n_periodic = 0;
  sum_time_periodic = 0;
  sum_cycle_periodic = 0;
  sum_n_event = 0;
  sys_mon.periodic_time_min = 0xFFFF;
  sys_mon.periodic_time_max = 0;
  sys_mon.periodic_cycle_min = 0xFFFF;
  sys_mon.periodic_cycle_max = 0;
}

void periodic_sysmon(void)
{
  /** Estimate periodic task cycle time */
  uint32_t periodic_usec = SysTimeTimer(periodic_timer);
  SysTimeTimerStart(periodic_timer);
  sum_time_periodic += periodic_usec;

  /* only periodic cycle : periodic_cycle = periodic_usec - sum_time_event; */
  periodic_cycle = periodic_usec - n_event * min_time_event;
  sum_cycle_periodic += periodic_cycle;

  /* remember min and max periodic times */
  if (periodic_usec < sys_mon.periodic_time_min) {
    sys_mon.periodic_time_min = periodic_usec;
  }
  if (periodic_usec > sys_mon.periodic_time_max) {
    sys_mon.periodic_time_max = periodic_usec;
  }

  /* remember min and max periodic cycle times */
  if (periodic_cycle < sys_mon.periodic_cycle_min) {
    sys_mon.periodic_cycle_min = periodic_cycle;
  }
  if (periodic_cycle > sys_mon.periodic_cycle_max) {
    sys_mon.periodic_cycle_max = periodic_cycle;
  }

  n_periodic++;
  sum_n_event += n_event;
  n_event = 0;
  sum_time_event = 0;
}

void event_sysmon(void)
{
  /** Store event calls total time and number of calls between two periodic calls */
  if (n_event > 0) {
    uint32_t t = SysTimeTimer(event_timer);
    if (t < min_time_event) {
      min_time_event = t;
    }
    sum_time_event += t;
  }
  SysTimeTimerStart(event_timer);
  n_event++;
}

