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
#define SYS_TIME_NB_TIMER 8
#endif

typedef uint8_t tid_t; ///< sys_time timer id type
typedef void (*sys_time_cb) (uint8_t id);

struct sys_time_timer {
  bool_t          in_use;
  sys_time_cb     cb;
  volatile bool_t elapsed;
  uint32_t        end_time; ///< in SYS_TICKS
  uint32_t        duration; ///< in SYS_TICKS
};

struct sys_time {
  volatile uint32_t nb_sec;       ///< full seconds since startup
  volatile uint32_t nb_sec_rem;   ///< remainder of second in CPU_TICKS
  volatile uint32_t nb_tick;      ///< in SYS_TICKS with SYS_TIME_RESOLUTION
  struct sys_time_timer timer[SYS_TIME_NB_TIMER];
};

extern struct sys_time sys_time;

//FIXME temporary hack
#define cpu_time_sec sys_time.nb_sec
#define cpu_time_ticks sys_time.nb_sec_rem

extern void sys_time_init(void);

/**
 * Register a new system timer.
 * @param duration Duration in seconds until the timer elapses.
 * @param cb Callback function that is called from the ISR when timer elapses, or NULL
 * @return -1 if it failed, the timer id otherwise
 */
extern int sys_time_register_timer(float duration, sys_time_cb cb);

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

static inline bool_t sys_time_check_and_ack_timer(tid_t id) {
  if (sys_time.timer[id].elapsed) {
    sys_time.timer[id].elapsed = FALSE;
    return TRUE;
  }
  return FALSE;
}

#define GET_CUR_TIME_FLOAT() ((float)sys_time.nb_sec + SEC_OF_CPU_TICKS((float)sys_time.nb_sec_rem))


/* CPU clock */
#define CPU_TICKS_OF_USEC(us) CPU_TICKS_OF_SEC((us) * 1e-6)
#define CPU_TICKS_OF_NSEC(ns) CPU_TICKS_OF_SEC((ns) * 1e-9)
#define SIGNED_CPU_TICKS_OF_USEC(us) SIGNED_CPU_TICKS_OF_SEC((us) * 1e-6)
#define SIGNED_CPU_TICKS_OF_NSEC(us) SIGNED_CPU_TICKS_OF_SEC((us) * 1e-9)

#define CPU_TICKS_PER_SEC CPU_TICKS_OF_SEC( 1.)


/* paparazzi sys_time timers */
#ifndef SYS_TIME_RESOLUTION
#define SYS_TIME_RESOLUTION ( 1./1024. )
#endif
#define SYS_TIME_RESOLUTION_CPU_TICKS CPU_TICKS_OF_SEC(SYS_TIME_RESOLUTION)

#define SYS_TIME_TICKS_OF_SEC(s) (uint32_t)((s) / SYS_TIME_RESOLUTION + 0.5)
#define SYS_TIME_TICKS_OF_USEC(us) SYS_TIME_TICKS_OF_SEC((us) * 1e-6)
#define SYS_TIME_TICKS_OF_NSEC(ns) SYS_TIME_TICKS_OF_SEC((ns) * 1e-9)

#define SEC_OF_SYS_TIME_TICKS(t) ((t) * SYS_TIME_RESOLUTION)
#define USEC_OF_SYS_TIME_TICKS(t) ((t) * SYS_TIME_RESOLUTION / 1e-6)
#define NSEC_OF_SYS_TIME_TICKS(t) ((t) * SYS_TIME_RESOLUTION / 1e-9)

#define USEC_OF_SEC(sec) ((sec) * 1e6)


#include "mcu_periph/sys_time_arch.h"

/* architecture specific init implementation */
extern void sys_time_arch_init(void);


#endif /* SYS_TIME_H */
