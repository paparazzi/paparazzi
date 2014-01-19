/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
/**
 * @file arch/chibios/mcu_periph/sys_time_arch.c
 * Implementation of system time functions for ChibiOS arch
 *
 * Mostly empty functions for Paparazzi compatibility,
 * since ChibiOS uses different system time functions.
 */
#include "mcu_periph/sys_time_arch.h"

/*
 * Extra defines for ChibiOS CPU monitoring
 */
uint32_t core_free_memory;
uint8_t thread_counter;
uint32_t cpu_counter;
uint32_t idle_counter;
uint8_t cpu_frequency;

void sys_time_arch_init( void ) {
  core_free_memory = 0;
  thread_counter = 0;
  cpu_counter = 0;
  idle_counter = 0;
  cpu_frequency = 0;
}

static inline uint32_t get_sys_time_usec(void) {
  return  (uint32_t)(chTimeNow()/CH_FREQUENCY*1000000);
}

static inline void sys_time_usleep(uint32_t us) {
  chThdSleep(US2ST(us));
}
