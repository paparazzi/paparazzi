/*
 * $Id:  $
 *  
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

#include "sys_mon.h"
#include "sys_time.h"

/* Global vars */
uint8_t cpu_load;
uint16_t periodic_time, periodic_cycle, periodic_cycle_min, periodic_cycle_max;
uint16_t event_time, event_number;

/* Local vars */
uint16_t n_periodic, n_event;
uint32_t time_periodic, time_event;
uint32_t sum_time_periodic, sum_cycle_periodic, sum_time_event, sum_n_event;

void init_sysmon(void) {
  cpu_load = 0;
  periodic_time = 0;
  periodic_cycle = 0;
  periodic_cycle_min = 0xFFFF;
  periodic_cycle_max = 0;
  event_time = 0;
  event_number = 0;

  n_periodic = 0;
  n_event = 0;
  sum_time_periodic = 0;
  sum_cycle_periodic = 0;
  sum_time_event = 0;
  sum_n_event = 0;
}

#include "uart.h"
#include "messages.h"
#include "downlink.h"

void periodic_report_sysmon(void) {
  /** Report system status at low frequency */
  if (n_periodic > 0) {
    periodic_time = Max(sum_time_periodic / n_periodic, 1);
    periodic_cycle = sum_cycle_periodic / n_periodic;
    cpu_load = 100 * periodic_cycle / periodic_time;
    event_number = sum_n_event / n_periodic;

    DOWNLINK_SEND_SYS_MON(DefaultChannel,&periodic_time,&periodic_cycle,&periodic_cycle_min,&periodic_cycle_max,&event_number,&cpu_load);
  }

  n_periodic = 0;
  sum_time_periodic = 0;
  sum_cycle_periodic = 0;
  sum_n_event = 0;
  periodic_cycle_min = 0xFFFF;
  periodic_cycle_max = 0;
}

void periodic_sysmon(void) {
  /** Estimate periodic task cycle time */
  SysTimeTimerStop(time_periodic);
  periodic_time = USEC_OF_SYS_TICS(time_periodic);
  periodic_cycle = periodic_time - sum_time_event /* - sum_time_event/n_event */;
  if (periodic_cycle < periodic_cycle_min) periodic_cycle_min = periodic_cycle;
  if (periodic_cycle > periodic_cycle_max) periodic_cycle_max = periodic_cycle;
  sum_time_periodic += periodic_time;
  sum_cycle_periodic += periodic_cycle;
  n_periodic++;
  sum_n_event += n_event;
  n_event = 0;
  sum_time_event = 0;
  SysTimeTimerStart(time_periodic);
}

void event_sysmon(void) {
  /** Store event calls total time and number of calls between two periodic calls */
  if (n_event > 0) {
    sum_time_event += USEC_OF_SYS_TICS(SysTimeTimer(time_event));
  }
  SysTimeTimerStart(time_event);
  n_event++;
}

