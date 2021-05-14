/*
 *
 * Copyright (C) 2009-2011 The Paparazzi Team
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

/**
 * @file mcu_periph/sys_time.h
 * @brief Architecture independent timing functions.
 *
 */

#ifndef SYS_TIME_H
#define SYS_TIME_H

#include <inttypes.h>
#include <stdlib.h>
#include "std.h"

#include BOARD_CONFIG

#ifndef SYS_TIME_NB_TIMER
#define SYS_TIME_NB_TIMER 16
#endif


/**
 * (Default) sys_time timer frequency in Hz.
 * sys_time.resolution is set from this define.
 */
#ifndef SYS_TIME_FREQUENCY
#if USE_CHIBIOS_RTOS
#define SYS_TIME_FREQUENCY CH_CFG_ST_FREQUENCY
#else /* NO RTOS */
#if defined PERIODIC_FREQUENCY
#define SYS_TIME_FREQUENCY (2 * PERIODIC_FREQUENCY)
#else /* !defined PERIODIC_FREQUENCY */
#define SYS_TIME_FREQUENCY 1000
#endif
#endif /* USE_CHIBIOS_RTOS */
#endif

typedef int8_t tid_t; ///< sys_time timer id type
typedef void (*sys_time_cb)(uint8_t id);

struct sys_time_timer {
  bool          in_use;
  sys_time_cb     cb;
  volatile bool elapsed;
  uint32_t        end_time; ///< in SYS_TIME_TICKS
  uint32_t        duration; ///< in SYS_TIME_TICKS
};

struct sys_time {
  volatile uint32_t nb_sec;       ///< full seconds since startup
  volatile uint32_t nb_sec_rem;   ///< remainder of seconds since startup in CPU_TICKS
  volatile uint32_t nb_tick;      ///< SYS_TIME_TICKS since startup
  struct sys_time_timer timer[SYS_TIME_NB_TIMER];

  float resolution;               ///< sys_time_timer resolution in seconds
  uint32_t ticks_per_sec;         ///< sys_time ticks per second (SYS_TIME_FREQUENCY)
  uint32_t resolution_cpu_ticks;  ///< sys_time_timer resolution in cpu ticks
  uint32_t cpu_ticks_per_sec;     ///< cpu ticks per second
};

extern struct sys_time sys_time;


extern void sys_time_init(void);

/**
 * Register a new system timer.
 * @param duration Duration in seconds until the timer elapses.
 * @param cb Callback function that is called from the ISR when timer elapses, or NULL
 * @return -1 if it failed, the timer id otherwise
 */
extern tid_t sys_time_register_timer(float duration, sys_time_cb cb);

/**
 * Register a new system timer with an fixed offset from another one.
 * @param timer timer providing start time and duration
 * @param offset offset in seconds beetween the timers (will overlap if longer than duration)
 * @param cb Callback function that is called from the ISR when timer elapses, or NULL
 * @return -1 if it failed, the timer id otherwise
 */
extern tid_t sys_time_register_timer_offset(tid_t timer, float offset, sys_time_cb cb);

/**
 * Cancel a system timer by id.
 * @param id Timer id.
 */
extern void sys_time_cancel_timer(tid_t id);

/**
 * Update the duration until a timer elapses.
 * @param id Timer id
 * @param duration Duration in seconds until the timer elapses.
 */
extern void sys_time_update_timer(tid_t id, float duration);

/**
 * Check if timer has elapsed.
 * @param id Timer id
 * @return TRUE if timer has elapsed
 */
static inline bool sys_time_check_and_ack_timer(tid_t id)
{
  if ((id < SYS_TIME_NB_TIMER) && (id >= 0)) {
    if (sys_time.timer[id].elapsed) {
        sys_time.timer[id].elapsed = false;
        return true;
    }
  }
  return false;
}

/**
 * Get the time in seconds since startup.
 * @return current system time as float with sys_time.resolution
 */
static inline float get_sys_time_float(void)
{
  return (float)(sys_time.nb_sec + (float)(sys_time.nb_sec_rem) / sys_time.cpu_ticks_per_sec);
}


/*
 * Convenience functions to convert between seconds and sys_time ticks.
 */
static inline uint32_t sys_time_ticks_of_sec(float seconds)
{
  return (uint32_t)(seconds * sys_time.ticks_per_sec + 0.5);
}

static inline uint32_t sys_time_ticks_of_msec(uint32_t msec)
{
  return msec * sys_time.ticks_per_sec / 1000;
}

static inline uint32_t sys_time_ticks_of_usec(uint32_t usec)
{
  return usec * sys_time.ticks_per_sec / 1000000;
}

static inline float sec_of_sys_time_ticks(uint32_t ticks)
{
  return (float)ticks * sys_time.resolution;
}

static inline uint32_t msec_of_sys_time_ticks(uint32_t ticks)
{
  return ticks * 1000 / sys_time.ticks_per_sec;
}

static inline uint32_t usec_of_sys_time_ticks(uint32_t ticks)
{
  return ticks * 1000 / sys_time.ticks_per_sec * 1000;
}



/*
 * Convenience functions to convert between seconds and CPU ticks.
 */
static inline uint32_t cpu_ticks_of_sec(float seconds)
{
  return (uint32_t)(seconds * sys_time.cpu_ticks_per_sec + 0.5);
}

static inline uint32_t cpu_ticks_of_usec(uint32_t usec)
{
  return usec * (sys_time.cpu_ticks_per_sec / 1000000);
}

static inline int32_t signed_cpu_ticks_of_usec(int32_t usec)
{
  return usec * ((int32_t)sys_time.cpu_ticks_per_sec / 1000000);
}

static inline uint32_t cpu_ticks_of_nsec(uint32_t nsec)
{
  return nsec * (sys_time.cpu_ticks_per_sec / 1000000) / 1000;
}

static inline uint32_t msec_of_cpu_ticks(uint32_t cpu_ticks)
{
  return cpu_ticks / (sys_time.cpu_ticks_per_sec / 1000);
}

static inline uint32_t usec_of_cpu_ticks(uint32_t cpu_ticks)
{
  return cpu_ticks / (sys_time.cpu_ticks_per_sec / 1000000);
}

static inline uint32_t nsec_of_cpu_ticks(uint32_t cpu_ticks)
{
  return cpu_ticks / (sys_time.cpu_ticks_per_sec / 1000000) / 1000;
}



#define USEC_OF_SEC(sec) ((sec) * 1e6)

#include "mcu_periph/sys_time_arch.h"

/* architecture specific init implementation */
extern void sys_time_arch_init(void);

/* Generic timer macros */
#define SysTimeTimerStart(_t) { _t = get_sys_time_usec(); }
#define SysTimeTimer(_t) ( get_sys_time_usec() - (_t))
#define SysTimeTimerStop(_t) { _t = ( get_sys_time_usec() - (_t)); }

#endif /* SYS_TIME_H */
