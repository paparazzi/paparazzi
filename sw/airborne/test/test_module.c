/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 */

/** @file test_module.c
 *  @brief Run one or multiple modules.
 * Simply calls the periodic and event functions of all added modules.
 *
 */

#include BOARD_CONFIG

#define DATALINK_C
#define MODULES_C
#define ABI_C

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "subsystems/datalink/downlink.h"
#include "generated/modules.h"
#include "subsystems/abi.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

/* MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

tid_t modules_tid;       ///< id for modules_periodic_task() timer

int main(void)
{
  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    if (sys_time_check_and_ack_timer(modules_tid)) {
      modules_periodic_task();
    }
    main_event_task();
  }

  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  downlink_init();

  modules_init();

  modules_tid = sys_time_register_timer(1. / MODULES_FREQUENCY, NULL);
}

static inline void main_periodic_task(void)
{
  LED_PERIODIC();
  RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});
}

static inline void main_event_task(void)
{
  mcu_event();
  modules_event_task();
}
