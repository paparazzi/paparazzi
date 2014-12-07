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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>

#include "std.h"

void mcu_arch_init(void) {
#if LUFTBOOT
PRINT_CONFIG_MSG("We are running luftboot, the interrupt vector is being relocated.")
#if defined STM32F4
  SCB_VTOR = 0x00004000;
#else
  SCB_VTOR = 0x00002000;
#endif
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
#elif EXT_CLK == 25000000
#if defined(STM32F4)
PRINT_CONFIG_MSG("Using 25MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&hse_25mhz_3v3[CLOCK_3V3_168MHZ]);
#endif
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

#if defined(STM32F1)
#define RCC_CFGR_PPRE2_SHIFT			11
#define RCC_CFGR_PPRE2				(7 << RCC_CFGR_PPRE2_SHIFT)

#define RCC_CFGR_PPRE1_SHIFT			8
#define RCC_CFGR_PPRE1				(7 << RCC_CFGR_PPRE1_SHIFT)

static inline uint32_t rcc_get_ppre1(void)
{
  return RCC_CFGR & RCC_CFGR_PPRE1;
}

static inline uint32_t rcc_get_ppre2(void)
{
  return RCC_CFGR & RCC_CFGR_PPRE2;
}
#elif defined(STM32F4)
static inline uint32_t rcc_get_ppre1(void)
{
  return RCC_CFGR &((1 << 10) | (1 << 11) | (1 << 12));
}

static inline uint32_t rcc_get_ppre2(void)
{
  return RCC_CFGR &((1 << 13) | (1 << 14) | (1 << 15));
}
#endif

/** @brief Get Timer clock frequency (before prescaling)
 * Only valid if using the internal clock for the timer.
 * Currently implemented for STM32F1x and STM32F405xx/407xx STM32F415xx/417xx.
 * Not valid for STM32F42xxx and STM32F43xxx.
 * @param[in] timer_peripheral Unsigned int32. Timer register address base
 * @return Unsigned int32. Timer base frequency
 */
uint32_t timer_get_frequency(uint32_t timer_peripheral)
{
  switch (timer_peripheral) {
    // Timers on APB1
    case TIM1:
    case TIM8:
#ifdef TIM9
    case TIM9:
#endif
#ifdef TIM10
    case TIM10:
#endif
#ifdef TIM11
    case TIM11:
#endif
      if (!rcc_get_ppre2())
        // no APB2 prescaler
        return rcc_ppre2_frequency;
      else
        return rcc_ppre2_frequency * 2;

    // timers on APB2
    case TIM2:
    case TIM3:
    case TIM4:
    case TIM5:
    case TIM6:
    case TIM7:
#ifdef TIM12
    case TIM12:
#endif
#ifdef TIM13
    case TIM13:
#endif
#ifdef TIM14
    case TIM14:
#endif
      if (!rcc_get_ppre1())
        // no APB2 prescaler
        return rcc_ppre1_frequency;
      else
        return rcc_ppre1_frequency * 2;
    default:
      // other timers currently not supported
      break;
  }
  return 0;
}
