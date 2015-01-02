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
 * @file mcu_periph/sys_time.c
 * @brief Architecture independent timing functions.
 *
 */

#include "mcu_periph/sys_time.h"
#include "mcu.h"

PRINT_CONFIG_VAR(SYS_TIME_FREQUENCY)

struct sys_time sys_time;

int sys_time_register_timer(float duration, sys_time_cb cb)
{

  uint32_t start_time = sys_time.nb_tick;
  for (int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (!sys_time.timer[i].in_use) {
      sys_time.timer[i].cb         = cb;
      sys_time.timer[i].elapsed    = FALSE;
      sys_time.timer[i].end_time   = start_time + sys_time_ticks_of_sec(duration);
      sys_time.timer[i].duration   = sys_time_ticks_of_sec(duration);
      sys_time.timer[i].in_use     = TRUE;
      return i;
    }
  }
  return -1;
}


void sys_time_cancel_timer(tid_t id)
{
  sys_time.timer[id].in_use     = FALSE;
  sys_time.timer[id].cb         = NULL;
  sys_time.timer[id].elapsed    = FALSE;
  sys_time.timer[id].end_time   = 0;
  sys_time.timer[id].duration   = 0;
}

// FIXME: race condition ??
void sys_time_update_timer(tid_t id, float duration)
{
  mcu_int_disable();
  sys_time.timer[id].end_time -= (sys_time.timer[id].duration - sys_time_ticks_of_sec(duration));
  sys_time.timer[id].duration = sys_time_ticks_of_sec(duration);
  mcu_int_enable();
}

void sys_time_init(void)
{
  sys_time.nb_sec     = 0;
  sys_time.nb_sec_rem = 0;
  sys_time.nb_tick    = 0;

  sys_time.ticks_per_sec = SYS_TIME_FREQUENCY;
  sys_time.resolution = 1.0 / sys_time.ticks_per_sec;

  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    sys_time.timer[i].in_use     = FALSE;
    sys_time.timer[i].cb         = NULL;
    sys_time.timer[i].elapsed    = FALSE;
    sys_time.timer[i].end_time   = 0;
    sys_time.timer[i].duration   = 0;
  }

  sys_time_arch_init();
}
