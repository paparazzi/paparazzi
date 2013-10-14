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
 * @brief chibios arch dependant implementation of sys time functions
 * @note Partially implemented (no CPU monitor), some extra variables
 * 		 for monitoring number of threads and free memory.
 *
 */
#include "mcu_periph/sys_time.h"

size_t heap_fragments;
size_t heap_free_total;
uint32_t core_free_memory;
uint8_t thread_counter;
uint32_t cpu_counter;
uint32_t idle_counter;
float cpu_frequency;

void sys_time_arch_init( void ) {
  heap_fragments = 0;
  heap_free_total = 0;
  core_free_memory = 0;
  thread_counter = 0;
  cpu_counter = 0;
  idle_counter = 0;
  cpu_frequency = 0.0;
}

