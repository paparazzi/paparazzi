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
#include "mcu_periph/sys_time_arch.h"
#include <ch.h>

#if !CH_DBG_STATISTICS
#error CH_DBG_STATISTICS should be defined to TRUE to use this monitoring tool
#endif

struct rtos_mon_cpu_window {
  rttime_t thread_time[RTOS_MON_MAX_THREADS];        ///< Current cumulative thread counters.
  thread_t *thread_ref[RTOS_MON_MAX_THREADS];        ///< Current thread identities for matching samples.
  rttime_t prev_thread_time[RTOS_MON_MAX_THREADS];   ///< Previous cumulative thread counters.
  thread_t *prev_thread_ref[RTOS_MON_MAX_THREADS];   ///< Previous thread identities for delta lookup.
  uint8_t prev_thread_count;                         ///< Number of valid entries in previous arrays.
  rttime_t prev_isr_time;                            ///< Previous cumulative ISR counter.
  uint8_t prev_stats_valid;                          ///< True after the first full sampling pass.
};

static struct rtos_mon_cpu_window cpu_window;

static uint16_t get_stack_free(const thread_t *tp);
static rttime_t get_thread_delta(const thread_t *tp, rttime_t current_time);
static uint16_t get_thread_load_x10(rttime_t thread_ticks, rttime_t total_ticks);
static uint8_t get_cpu_load_percent(rttime_t idle_ticks, rttime_t total_ticks);

#if USE_SHELL
#include "modules/core/shell.h"
#include "mcu_periph/sys_time.h"
#include "printf.h"
#include "string.h"

typedef struct _ThreadCpuInfo {
  float    ticks[RTOS_MON_MAX_THREADS];
  float    cpu[RTOS_MON_MAX_THREADS];
  float    totalTicks;
  float    totalISRTicks;
} ThreadCpuInfo ;


static void stampThreadCpuInfo(ThreadCpuInfo *ti)
{
  const thread_t *tp =  chRegFirstThread();
  uint32_t idx = 0;

  ti->totalTicks = 0;
  do {
    ti->ticks[idx] = (float) tp->stats.cumulative;
    ti->totalTicks += ti->ticks[idx];
    tp = chRegNextThread((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < RTOS_MON_MAX_THREADS));
  ti->totalISRTicks = currcore->kernel_stats.m_crit_isr.cumulative;
  ti->totalTicks += ti->totalISRTicks;
  tp =  chRegFirstThread();
  idx = 0;
  do {
    ti->cpu[idx] = (ti->totalTicks > 0.f) ? ((ti->ticks[idx] * 100.f) / ti->totalTicks) : 0.f;
    tp = chRegNextThread((thread_t *)tp);
    idx++;
  } while ((tp != NULL) && (idx < RTOS_MON_MAX_THREADS));
}

static float stampThreadGetCpuPercent(const ThreadCpuInfo *ti, const uint32_t idx)
{
  if (idx >= RTOS_MON_MAX_THREADS) {
    return -1.f;
  }

  return ti->cpu[idx];
}

static float stampISRGetCpuPercent(const ThreadCpuInfo *ti)
{
  return (ti->totalTicks > 0.f) ? (ti->totalISRTicks * 100.0f / ti->totalTicks) : 0.f;
}

static void cmd_threads(BaseSequentialStream *lchp, int argc, const char *const argv[])
{
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp = chRegFirstThread();
  (void)argv;
  (void)argc;
  float totalTicks = 0;
  float idleTicks = 0;

  static ThreadCpuInfo threadCpuInfo = {
    .ticks = {[0 ... RTOS_MON_MAX_THREADS - 1] = 0.f},
    .cpu =   {[0 ... RTOS_MON_MAX_THREADS - 1] = -1.f},
    .totalTicks = 0.f,
    .totalISRTicks = 0.f
  };

  stampThreadCpuInfo(&threadCpuInfo);

  chprintf(lchp, "    addr    stack  frestk prio refs  state        time \t percent        name\r\n");
  uint32_t idx = 0;
  do {
    chprintf(lchp, "%.8lx %.8lx %6lu %4lu %4lu %9s %9lu   %.2f%%    \t%s\r\n",
             (uint32_t)tp, (uint32_t)tp->ctx.sp,
             get_stack_free(tp),
             (uint32_t)tp->hdr.pqueue.prio, (uint32_t)(tp->refs - 1),
             states[tp->state],
             (uint32_t)RTC2MS(STM32_SYSCLK, tp->stats.cumulative),
             stampThreadGetCpuPercent(&threadCpuInfo, idx),
             chRegGetThreadNameX(tp));

    totalTicks += (float)tp->stats.cumulative;
    if (strcmp(chRegGetThreadNameX(tp), "idle") == 0) {
      idleTicks = (float)tp->stats.cumulative;
    }
    tp = chRegNextThread((thread_t *)tp);
    idx++;
  } while (tp != NULL);

  totalTicks += currcore->kernel_stats.m_crit_isr.cumulative;
  const float idlePercent = (totalTicks > 0.f) ? ((idleTicks * 100.f) / totalTicks) : 0.f;
  const float cpuPercent = 100.f - idlePercent;
  chprintf(lchp, "Interrupt Service Routine \t\t     %9lu   %.2f%%    \tISR\r\n",
           (uint32_t)RTC2MS(STM32_SYSCLK, threadCpuInfo.totalISRTicks),
           stampISRGetCpuPercent(&threadCpuInfo));
  chprintf(lchp, "\r\ncpu load = %.2f%%\r\n", cpuPercent);
}

static void cmd_rtos_mon(shell_stream_t *sh, int argc, const char *const argv[])
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
  memory_area_t area;
  total_fragments = chHeapStatus(NULL, &total_fragmented_free_space, &largest_free_block);
  chCoreGetStatusX(&area);

  rtos_mon.core_free_memory = area.size;
  rtos_mon.heap_fragments = total_fragments;
  rtos_mon.heap_largest = largest_free_block;
  rtos_mon.heap_free_memory = total_fragmented_free_space;
  rtos_mon.thread_counter = 0;

  // loop threads to find idle thread
  // store info on other threads
  thread_t *tp;
  rttime_t idle_counter = 0;
  rttime_t sum = 0;
  rttime_t isr_time = currcore->kernel_stats.m_crit_isr.cumulative;
  rtos_mon.thread_name_idx = 0;
  tp = chRegFirstThread();
  do {
    const char *name = chRegGetThreadNameX(tp);
    if (name == NULL) {
      name = "";
    }

    // add beginning of thread name to buffer
    for (i = 0; i < RTOS_MON_NAME_LEN - 1 && name[i] != '\0'; i++) {
      rtos_mon.thread_names[rtos_mon.thread_name_idx++] = name[i];
    }
    rtos_mon.thread_names[rtos_mon.thread_name_idx++] = ';';

    // store free stack for this thread
    rtos_mon.thread_free_stack[rtos_mon.thread_counter] = get_stack_free(tp);

    // store cumulative time spent in thread
    cpu_window.thread_time[rtos_mon.thread_counter] = tp->stats.cumulative;
    rttime_t thread_delta = get_thread_delta(tp, cpu_window.thread_time[rtos_mon.thread_counter]);
    sum += thread_delta;

    // if current thread is the idle thread, store its sampled value separately
    if (tp == chSysGetIdleThreadX()) {
      idle_counter = thread_delta;
    }
    // get next thread
    cpu_window.thread_ref[rtos_mon.thread_counter] = tp;
    tp = chRegNextThread(tp);
    // increment thread counter
    rtos_mon.thread_counter++;
  } while (tp != NULL && rtos_mon.thread_counter < RTOS_MON_MAX_THREADS);
  rtos_mon.thread_names[rtos_mon.thread_name_idx] = '\0';

  // sum the time spent in ISR
  rttime_t isr_delta = cpu_window.prev_stats_valid ? (isr_time - cpu_window.prev_isr_time) : 0;
  sum += isr_delta;

  // store individual thread load (as deci-percent integer, i.e. 10 * %)
  for (i = 0; i < rtos_mon.thread_counter; i ++) {
    rtos_mon.thread_load[i] = get_thread_load_x10(
      get_thread_delta(cpu_window.thread_ref[i], cpu_window.thread_time[i]), sum);
    cpu_window.prev_thread_ref[i] = cpu_window.thread_ref[i];
    cpu_window.prev_thread_time[i] = cpu_window.thread_time[i];
  }

  rtos_mon.cpu_load = get_cpu_load_percent(idle_counter, sum);
  cpu_window.prev_thread_count = rtos_mon.thread_counter;
  cpu_window.prev_isr_time = isr_time;
  cpu_window.prev_stats_valid = 1;
}

static uint16_t get_stack_free(const thread_t *tp)
{
  int32_t index = 0;
  extern const uint8_t __ram0_end__;
  unsigned long long *stkAdr = (unsigned long long *)((uint8_t *) tp->wabase);
  while ((stkAdr[index] == 0x5555555555555555) && (((uint8_t *) & (stkAdr[index])) < &__ram0_end__)) {
    index++;
  }
  const int32_t freeBytes =  index * (int32_t) sizeof(long long);
  return (uint16_t)freeBytes;
}

static rttime_t get_thread_delta(const thread_t *tp, rttime_t current_time)
{
  // Return the sampled runtime accumulated by a thread since the previous
  // monitoring pass. New threads report zero until they exist in both samples.
  if (!cpu_window.prev_stats_valid) {
    return 0;
  }

  for (uint8_t i = 0; i < cpu_window.prev_thread_count; i++) {
    if (cpu_window.prev_thread_ref[i] == tp) {
      return current_time - cpu_window.prev_thread_time[i];
    }
  }

  return 0;
}

static uint16_t get_thread_load_x10(rttime_t thread_ticks, rttime_t total_ticks)
{
  // Convert a thread runtime delta into deci-percent of the total sampled
  // runtime window so the result fits the telemetry representation.
  if (total_ticks == 0) {
    return 0;
  }

  return (uint16_t)((1000ULL * (uint64_t)thread_ticks) / (uint64_t)total_ticks);
}

static uint8_t get_cpu_load_percent(rttime_t idle_ticks, rttime_t total_ticks)
{
  // Compute global CPU load over the sampled window from the non-idle share
  // of the measured runtime, including ISR time in the total budget.
  if (total_ticks == 0) {
    return 0;
  }

  return (uint8_t)((100ULL * (uint64_t)(total_ticks - idle_ticks)) / (uint64_t)total_ticks);
}
