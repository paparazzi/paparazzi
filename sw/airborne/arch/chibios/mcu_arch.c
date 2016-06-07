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
 * @file arch/chibios/mcu_arch.c
 * Microcontroller initialization function for ChibiOS
 *
 * ChibiOS initialized peripherals by itself, only interrupt
 * vector has to be relocated for Luftboot
 */

/* ChibiOS includes */
#include "ch.h"
#include "hal.h"
/* Paparazzi includes */
#include "mcu.h"


/*
 * SCB_VTOR has to be relocated if Luftboot is used
 */
void mcu_arch_init(void)
{
#if LUFTBOOT
  PRINT_CONFIG_MSG("We are running luftboot, the interrupt vector is being relocated.")
#if defined STM32F4
  PRINT_CONFIG_MSG("STM32F4")
  SCB_VTOR = 0x00004000;
#else
  PRINT_CONFIG_MSG("STM32F1")
  SCB_VTOR = 0x00002000;
#endif
#endif

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
}
