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
 * @file "arch/chibios/modules/core/rtos_mon_arch.c"
 * @author Gautier Hattenberger
 * RTOS monitoring tool
 * ChibiOS implementation
 */

#include "modules/core/sys_mon_rtos.h"
#include <ch.h>

#if !CH_DBG_THREADS_PROFILING
#error CH_DBG_THREADS_PROFILING should be defined to TRUE to use this monitoring tool
#endif

static uint32_t thread_p_time[RTOS_MON_MAX_THREADS];
static uint32_t idle_counter, last_idle_counter;

static uint16_t get_stack_free(const thread_t *tp);

void rtos_mon_init_arch(void)
{
  idle_counter = 0;
  last_idle_counter = 0;
}

// Fill data structure
void rtos_mon_periodic_arch(void)
{
  int i;
  size_t total_fragments, total_fragmented_free_space, largest_free_block;
  total_fragments = chHeapStatus(NULL, &total_fragmented_free_space, &largest_free_block);

  rtos_mon.core_free_memory = chCoreGetStatusX();
  rtos_mon.heap_fragments = total_fragments;
  rtos_mon.heap_largest = largest_free_block;
  rtos_mon.heap_free_memory = total_fragmented_free_space;
  rtos_mon.thread_counter = 0;

  // loop threads to find idle thread
  // store info on other threads
  thread_t *tp;
  float sum = 0.f;
  rtos_mon.thread_name_idx = 0;
  tp = chRegFirstThread();
  do {
    // add beginning of thread name to buffer
    for (i = 0; i < RTOS_MON_NAME_LEN-1 && tp->name[i] != '\0'; i++) {
      rtos_mon.thread_names[rtos_mon.thread_name_idx++] = tp->name[i];
    }
    rtos_mon.thread_names[rtos_mon.thread_name_idx++] = ';';

    // store free stack for this thread
    rtos_mon.thread_free_stack[i] = get_stack_free(tp);

    // store time spend in thread
    thread_p_time[rtos_mon.thread_counter] = tp->time;
    sum += (float)(tp->time);

    // if current thread is 'idle' thread, store its value separately
    if (tp == chSysGetIdleThreadX()) {
      idle_counter = (uint32_t)tp->time;
    }
    // get next thread
    tp = chRegNextThread(tp);
    // increment thread counter
    rtos_mon.thread_counter++;
  } while (tp != NULL && rtos_mon.thread_counter < RTOS_MON_MAX_THREADS);

  // store individual thread load (as centi-percent integer)
  for (i = 0; i < rtos_mon.thread_counter; i ++) {
    rtos_mon.thread_load[i] = (uint16_t)(10000.f * (float)thread_p_time[i] / sum);
  }

  // assume we call the counter once a second
  // so the difference in seconds is always one
  // NOTE: not perfectly precise, +-5% on average so take it into consideration
  rtos_mon.cpu_load = (1 - (float)(idle_counter - last_idle_counter) / CH_CFG_ST_FREQUENCY) * 100;
  last_idle_counter = idle_counter;
}

static uint16_t get_stack_free(const thread_t *tp)
{
#if defined STM32F4
  int32_t index = 0;
  // FIXME this is for STM32F4 only
  const uint8_t *max_ram_addr = (uint8_t*) (0x20000000 + (128*1024));
  const int32_t internal_struct_size = (CH_KERNEL_MAJOR == 2) ? 80 : 120;
  unsigned long long *stk_addr = (unsigned long long *) ((uint8_t *)tp + internal_struct_size);

  while ((stk_addr[index] == 0x5555555555555555) && (((uint8_t *) &(stk_addr[index])) < max_ram_addr))
    index++;

  const int32_t free_bytes = (index * (int32_t)sizeof(long long)) - internal_struct_size;
  uint16_t ret;
  if (free_bytes < 0) {
    // stack overflow ?
    ret = 0;
  } else if (free_bytes > 0xFFFF) {
    // return value in Kbytes
    ret = (uint16_t) (free_bytes / 1024);
  } else {
    // return value in bytes
    ret = (uint16_t) free_bytes;
  }
  return ret;
#else
  (void)tp;
  return 0;
#endif
}

