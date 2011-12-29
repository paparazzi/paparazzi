/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2009 Pascal Brisset, Antoine Drouin
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

/*
 *\brief architecture independant timing functions
 *
 */

#ifndef SYS_TIME_H
#define SYS_TIME_H

#include <inttypes.h>
#include "std.h"

#include BOARD_CONFIG

#ifndef SYS_TIME_NB_TIMER
#define  SYS_TIME_NB_TIMER 5
#endif

typedef void (*sys_time_cb) (uint8_t id);

struct sys_time_timer {
  bool_t          in_use;
  sys_time_cb     cb;
  volatile bool_t elapsed;
  uint32_t        end_time;
  uint32_t        duration;
};

struct sys_time {
  uint32_t nb_sec;
  uint32_t nb_sec_rem;
  uint32_t nb_tic;
  struct sys_time_timer timer[SYS_TIME_NB_TIMER];
};

extern struct sys_time sys_time;

extern void sys_time_init(void);
extern uint8_t sys_time_register_timer(uint32_t duration, sys_time_cb cb);
extern void    sys_time_cancel_timer(uint8_t id);
extern void    sys_time_update_timer(uint8_t id, uint32_t duration);

static inline bool_t sys_time_check_and_ack_timer( uint8_t id ) {
  if (sys_time.timer[id].elapsed) {
    sys_time.timer[id].elapsed = FALSE;
    return TRUE;
  }
  return FALSE;
}

#ifndef SYS_TIME_RESOLUTION
#define SYS_TIME_RESOLUTION SYS_TIME_TICS_OF_SEC( 1./1048576.)
#endif

#define SYS_TIME_TIMER_S(_s) (SYS_TIME_TICS_OF_SEC(_s)/SYS_TIME_RESOLUTION)

#define SYS_TIME_TICS_OF_USEC(us) SYS_TIME_TICS_OF_SEC((us) * 1e-6)
#define SYS_TIME_TICS_OF_NSEC(ns) SYS_TIME_TICS_OF_SEC((ns) * 1e-9)
#define SYS_TIME_SIGNED_TICS_OF_USEC(us) SYS_TIME_SIGNED_TICS_OF_SEC((us) * 1e-6)
#define SYS_TIME_SIGNED_TICS_OF_NSEC(us) SYS_TIME_SIGNED_TICS_OF_SEC((us) * 1e-9)

#define SYS_TIME_TICS_PER_SEC SYS_TIME_TICS_OF_SEC( 1.)

#include "mcu_periph/sys_time_arch.h"


#endif /* SYS_TIME_H */
