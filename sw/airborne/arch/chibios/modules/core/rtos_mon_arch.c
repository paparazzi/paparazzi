/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2020 Gautier Hattenberger, Alexandre Bustico
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

#if !CH_DBG_STATISTICS
#error CH_DBG_STATISTICS should be defined to TRUE to use this monitoring tool
#endif

static uint32_t thread_p_time[RTOS_MON_MAX_THREADS];

static uint16_t get_stack_free(const thread_t *tp);

#if USE_SHELL
#include "modules/core/shell.h"
#include "printf.h"
#include "string.h"

typedef struct _ThreadCpuInfo {
  float    ticks[RTOS_MON_MAX_THREADS];
  float    cpu[RTOS_MON_MAX_THREADS];
  float    totalTicks;
  float    totalISRTicks;
} ThreadCpuInfo ;


static void stampThreadCpuInfo (ThreadCpuInfo *ti)
{
  const thread_t *tp =  chRegFirstThread();
  uint32_t idx=0;

  ti->totalTicks =0;
  do {
    ti->ticks[idx] = (float) tp->stats.cumulative;
    ti->totalTicks += ti->ticks[idx];
    tp = chRegNextThread ((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < RTOS_MON_MAX_THREADS));
  ti->totalISRTicks = ch.kernel_stats.m_crit_isr.cumulative;
  ti->totalTicks += ti->totalISRTicks;
  tp =  chRegFirstThread();
  idx=0;
  do {
    ti->cpu[idx] =  (ti->ticks[idx]*100.f) / ti->totalTicks;
    tp = chRegNextThread ((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < RTOS_MON_MAX_THREADS));
}

static float stampThreadGetCpuPercent (const ThreadCpuInfo *ti, const uint32_t idx)
{
  if (idx >= RTOS_MON_MAX_THREADS)
    return -1.f;

  return ti->cpu[idx];
}

static float stampISRGetCpuPercent (const ThreadCpuInfo *ti)
{
  return ti->totalISRTicks * 100.0f / ti->totalTicks;
}

static void cmd_threads(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp = chRegFirstThread();
  (void)argv;
  (void)argc;
  float totalTicks=0;
  float idleTicks=0;

  static ThreadCpuInfo threadCpuInfo = {
    .ticks = {[0 ... RTOS_MON_MAX_THREADS-1] = 0.f},
    .cpu =   {[0 ... RTOS_MON_MAX_THREADS-1] =-1.f},
    .totalTicks = 0.f,
    .totalISRTicks = 0.f
  };

  stampThreadCpuInfo (&threadCpuInfo);

  chprintf (lchp, "    addr    stack  frestk prio refs  state        time \t percent        name\r\n");
  uint32_t idx=0;
  do {
    chprintf (lchp, "%.8lx %.8lx %6lu %4lu %4lu %9s %9lu   %.2f%%    \t%s\r\n",
	      (uint32_t)tp, (uint32_t)tp->ctx.sp,
	      get_stack_free (tp),
	      (uint32_t)tp->hdr.pqueue.prio, (uint32_t)(tp->refs - 1),
	      states[tp->state],
	      (uint32_t)RTC2MS(STM32_SYSCLK, tp->stats.cumulative),
	      stampThreadGetCpuPercent (&threadCpuInfo, idx),
	      chRegGetThreadNameX(tp));

    totalTicks+= (float)tp->stats.cumulative;
    if (strcmp(chRegGetThreadNameX(tp), "idle") == 0)
      idleTicks = (float)tp->stats.cumulative;
    tp = chRegNextThread ((thread_t *)tp);
    idx++;
  } while (tp != NULL);

  totalTicks += ch.kernel_stats.m_crit_isr.cumulative;
  const float idlePercent = (idleTicks*100.f)/totalTicks;
  const float cpuPercent = 100.f - idlePercent;
  chprintf (lchp, "Interrupt Service Routine \t\t     %9lu   %.2f%%    \tISR\r\n",
	    (uint32_t)RTC2MS(STM32_SYSCLK,threadCpuInfo.totalISRTicks),
	    stampISRGetCpuPercent(&threadCpuInfo));
  chprintf (lchp, "\r\ncpu load = %.2f%%\r\n", cpuPercent);
}

static void cmd_rtos_mon(shell_stream_t *sh, int argc, const char * const argv[])
{
  (void) argv;
  if (argc > 0) {
    chprintf(sh, "Usage: rtos_mon\r\n");
    return;
  }

  chprintf(sh, "Data reported in the RTOS_MON message:\r\n");
  chprintf(sh, " core free mem: %u\r\n", rtos_mon.core_free_memory);
  chprintf(sh, " heap free mem: %u\r\n", rtos_mon.heap_free_memory);
  chprintf(sh, " heap fragments: %u\r\n", rtos_mon.heap_fragments);
  chprintf(sh, " heap largest: %u\r\n", rtos_mon.heap_largest);
  chprintf(sh, " CPU load: %d \%\r\n", rtos_mon.cpu_load);
  chprintf(sh, " number of threads: %d\r\n", rtos_mon.thread_counter);
  chprintf(sh, " thread names: %s\r\n", rtos_mon.thread_names);
  for (int i = 0; i < rtos_mon.thread_counter; i++) {
    chprintf(sh, " thread %d load: %0.1f, free stack: %d\r\n", i,
        (float)rtos_mon.thread_load[i] / 10.f, rtos_mon.thread_free_stack[i]);
  }
  chprintf(sh, " CPU time: %.2f\r\n", rtos_mon.cpu_time);
}
#endif

void rtos_mon_init_arch(void)
{
#if USE_SHELL
  shell_add_entry("rtos_mon", cmd_rtos_mon);
  shell_add_entry("threads", cmd_threads);
#endif
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
  float idle_counter = 0.f;
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
    thread_p_time[rtos_mon.thread_counter] = tp->stats.cumulative;
    sum += (float)(tp->stats.cumulative);

    // if current thread is 'idle' thread, store its value separately
    if (tp == chSysGetIdleThreadX()) {
      idle_counter = (float)tp->stats.cumulative;
    }
    // get next thread
    tp = chRegNextThread(tp);
    // increment thread counter
    rtos_mon.thread_counter++;
  } while (tp != NULL && rtos_mon.thread_counter < RTOS_MON_MAX_THREADS);
  // sum the time spent in ISR
  sum += ch.kernel_stats.m_crit_isr.cumulative;
  // store individual thread load (as centi-percent integer, i.e. (th_time/sum)*10*100)
  for (i = 0; i < rtos_mon.thread_counter; i ++) {
    rtos_mon.thread_load[i] = (uint16_t)(1000.f * (float)thread_p_time[i] / sum);
  }

  // assume we call the counter once a second
  // so the difference in seconds is always one
  // NOTE: not perfectly precise, +-5% on average so take it into consideration
  rtos_mon.cpu_load = (uint8_t)((1.f - (idle_counter / sum)) * 100.f);
}

static uint16_t get_stack_free(const thread_t *tp)
{
  int32_t index = 0;
  extern const uint8_t __ram0_end__;
  unsigned long long *stkAdr =  (unsigned long long *) ((uint8_t *) tp->wabase);
  while ((stkAdr[index] == 0x5555555555555555) && ( ((uint8_t *) &(stkAdr[index])) < &__ram0_end__))
    index++;
  const int32_t freeBytes =  index * (int32_t) sizeof(long long);
  return (uint16_t)freeBytes;
}

