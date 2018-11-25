/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/core/rtos_mon.c"
 * @author Gautier Hattenberger
 * RTOS monitoring tool
 */

#include "modules/core/sys_mon.h"
#include "modules/core/sys_mon_rtos.h"
#include "subsystems/datalink/downlink.h"

struct rtos_monitoring rtos_mon;

void init_sysmon(void)
{
  // zero structure
  memset(&rtos_mon, 0, sizeof(struct rtos_monitoring));
  // arch init
  rtos_mon_init_arch();
}


// Periodic report of RTOS parameters
// This function is actually arch dependent
void periodic_report_sysmon(void)
{
  // update struct
  rtos_mon_periodic_arch();

  // update cpu time
  rtos_mon.cpu_time = get_sys_time_float();

  // send report
  DOWNLINK_SEND_RTOS_MON(DefaultChannel, DefaultDevice,
      &rtos_mon.thread_counter,
      &rtos_mon.cpu_load,
      &rtos_mon.core_free_memory,
      &rtos_mon.heap_free_memory,
      &rtos_mon.heap_fragments,
      &rtos_mon.heap_largest,
      &rtos_mon.cpu_time);

}

void periodic_sysmon(void) {}

void event_sysmon(void) {}
