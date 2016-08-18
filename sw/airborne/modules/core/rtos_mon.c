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
#include "modules/core/rtos_mon_arch.h"
#include "subsystems/datalink/downlink.h"
#include <string.h>

// Maximum number of threads
// The limit is related to the max size of the report message
#define RTOS_MON_MAX_THREADS 20

// Maximun len of a thread name (including trailing semi-colon)
// Currently only the first 4 char + ';' at most
#define RTOS_MON_NAME_LEN 5

// Names buffer size
#define RTOS_MON_THREAD_NAMES (RTOS_MON_MAX_THREADS * RTOS_MON_NAME_LEN)

// RTOS structure
struct rtos_monitoring {
  uint32_t core_free_memory;                        ///< core free memory in bytes
  uint32_t heap_free_memory;                        ///< heap free memory in bytes
  uint8_t cpu_load;                                 ///< global CPU/MCU load in %
  uint8_t thread_counter;                           ///< number of threads
  uint16_t thread_load[RTOS_MON_MAX_THREADS];       ///< individual thread load in centi-percent (10*%)
  uint16_t thread_free_stack[RTOS_MON_MAX_THREADS]; ///< individual thread free stack in bytes
  char thread_names[RTOS_MON_THREAD_NAMES+1];       ///< string of thread names / identifiers
  uint8_t thread_name_idx;                          ///< length of the string in thread_names buffer
  float cpu_time; // in secs since startup
};

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
      &rtos_mon.cpu_time);

}

void periodic_sysmon(void) {}

void event_sysmon(void) {}
