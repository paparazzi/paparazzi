/*
 * Copyright (C) 2010-2012 The Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/stm32/mcu_arch.c
 * @brief stm32 arch dependant microcontroller initialisation functions.
 * @ingroup stm32_arch
 */

#include "mcu.h"

#include BOARD_CONFIG

#include <inttypes.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>

#include "std.h"

void mcu_arch_init(void) {
#if LUFTBOOT
PRINT_CONFIG_MSG("We are running luftboot, the interrupt vector is being relocated.")
  SCB_VTOR = 0x00002000;
#endif
#if EXT_CLK == 8000000
#if defined(STM32F1)
PRINT_CONFIG_MSG("Using 8MHz external clock to PLL it to 72MHz.")
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif defined(STM32F4)
PRINT_CONFIG_MSG("Using 8MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
#endif
#elif EXT_CLK == 12000000
#if defined(STM32F1)
PRINT_CONFIG_MSG("Using 12MHz external clock to PLL it to 72MHz.")
  rcc_clock_setup_in_hse_12mhz_out_72mhz();
#elif defined(STM32F4)
PRINT_CONFIG_MSG("Using 12MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&hse_12mhz_3v3[CLOCK_3V3_168MHZ]);
#endif
#elif EXT_CLK == 16000000
#if defined(STM32F4)
PRINT_CONFIG_MSG("Using 16MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&hse_16mhz_3v3[CLOCK_3V3_168MHZ]);
#endif
#elif (INT_CLK == 8000000) && (AHB_CLK == 64000000) && defined(STM32F3)
 PRINT_CONFIG_MSG("Using 8MHz internal clock to PLL it to 64MHz.")
 rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
#elif (INT_CLK == 8000000) && (AHB_CLK == 48000000) && defined(STM32F3)
 PRINT_CONFIG_MSG("Using 8MHz internal clock to PLL it to 48MHz.")
 rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_48MHZ]);
#else
#error EXT_CLK is either set to an unsupported frequency or not defined at all. Please check!
#endif

  /* Configure priority grouping 0 bits for pre-emption priority and 4 bits for sub-priority.
   * this was previously in i2c driver
   * FIXME is it really needed ?
   */
#ifndef RTOS_IS_CHIBIOS
  scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_NOGROUP_SUB16);
#endif

}
