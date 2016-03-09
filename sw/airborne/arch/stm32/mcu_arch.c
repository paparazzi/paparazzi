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

#if defined(STM32F4)
/* untested, should go into libopencm3 */
const struct rcc_clock_scale rcc_hse_24mhz_3v3[RCC_CLOCK_3V3_END] = {
  { /* 48MHz */
    .pllm = 24,
    .plln = 96,
    .pllp = 2,
    .pllq = 2,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .power_save = 1,
    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_3WS,
    .ahb_frequency  = 48000000,
    .apb1_frequency = 12000000,
    .apb2_frequency = 24000000,
  },
  { /* 84MHz */
    .pllm = 24,
    .plln = 336,
    .pllp = 4,
    .pllq = 7,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_2,
    .ppre2 = RCC_CFGR_PPRE_DIV_NONE,
    .power_save = 1,
    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_2WS,
    .ahb_frequency  = 84000000,
    .apb1_frequency = 42000000,
    .apb2_frequency = 84000000,
  },
  { /* 120MHz */
    .pllm = 24,
    .plln = 240,
    .pllp = 2,
    .pllq = 5,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .power_save = 1,
    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_3WS,
    .ahb_frequency  = 120000000,
    .apb1_frequency = 30000000,
    .apb2_frequency = 60000000,
  },
  { /* 168MHz */
    .pllm = 24,
    .plln = 336,
    .pllp = 2,
    .pllq = 7,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE | FLASH_ACR_LATENCY_5WS,
    .ahb_frequency  = 168000000,
    .apb1_frequency = 42000000,
    .apb2_frequency = 84000000,
  },
};
#endif

#if defined(STM32F1)
/*---------------------------------------------------------------------------*/
/** @brief RCC Set System Clock HSE at 24MHz from HSE at 24MHz

*/
void rcc_clock_setup_in_hse_24mhz_out_24mhz_pprz(void);
void rcc_clock_setup_in_hse_24mhz_out_24mhz_pprz(void)
{
  /* Enable internal high-speed oscillator. */
  rcc_osc_on(RCC_HSI);
  rcc_wait_for_osc_ready(RCC_HSI);

  /* Select HSI as SYSCLK source. */
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

  /* Enable external high-speed oscillator 24MHz. */
  rcc_osc_on(RCC_HSE);
  rcc_wait_for_osc_ready(RCC_HSE);
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

  /*
   * Set prescalers for AHB, ADC, ABP1, ABP2.
   * Do this before touching the PLL (TODO: why?).
   */
  rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 24MHz Max. 72MHz */
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);  /* Set. 12MHz Max. 14MHz */
  rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_NODIV);    /* Set. 24MHz Max. 36MHz */
  rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 24MHz Max. 72MHz */

  /*
   * Sysclk runs with 24MHz -> 0 waitstates.
   * 0WS from 0-24MHz
   * 1WS from 24-48MHz
   * 2WS from 48-72MHz
   */
  flash_set_ws(FLASH_ACR_LATENCY_0WS);

  /*
   * Set the PLL multiplication factor to 2.
   * 24MHz (external) * 2 (multiplier) / 2 (RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2) = 24MHz
   */
  rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL2);

  /* Select HSE as PLL source. */
  rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

  /*
   * External frequency divide by 2 before entering PLL
   * (only valid/needed for HSE).
   */
  rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2);

  rcc_osc_on(RCC_PLL);
  rcc_wait_for_osc_ready(RCC_PLL);

  /* Select PLL as SYSCLK source. */
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

  /* Set the peripheral clock frequencies used */
  rcc_ahb_frequency = 24000000;
  rcc_apb1_frequency = 24000000;
  rcc_apb2_frequency = 24000000;
}
#endif

void mcu_arch_init(void)
{
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
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif
#elif EXT_CLK == 12000000
#if defined(STM32F1)
  PRINT_CONFIG_MSG("Using 12MHz external clock to PLL it to 72MHz.")
  rcc_clock_setup_in_hse_12mhz_out_72mhz();
#elif defined(STM32F4)
  PRINT_CONFIG_MSG("Using 12MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&rcc_hse_12mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif
#elif EXT_CLK == 16000000
#if defined(STM32F4)
  PRINT_CONFIG_MSG("Using 16MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#endif
#elif EXT_CLK == 24000000
#if defined(STM32F4)
  PRINT_CONFIG_MSG("Using 24MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&rcc_hse_24mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#elif defined(STM32F1)
  rcc_clock_setup_in_hse_24mhz_out_24mhz_pprz();
#endif
#elif EXT_CLK == 25000000
#if defined(STM32F4)
  PRINT_CONFIG_MSG("Using 25MHz external clock to PLL it to 168MHz.")
  rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
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
#define RCC_CFGR_PPRE2_SHIFT      11
#define RCC_CFGR_PPRE2        (7 << RCC_CFGR_PPRE2_SHIFT)

#define RCC_CFGR_PPRE1_SHIFT      8
#define RCC_CFGR_PPRE1        (7 << RCC_CFGR_PPRE1_SHIFT)

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
  return RCC_CFGR & ((1 << 10) | (1 << 11) | (1 << 12));
}

static inline uint32_t rcc_get_ppre2(void)
{
  return RCC_CFGR & ((1 << 13) | (1 << 14) | (1 << 15));
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
      // Timers on high speed APB2
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
      if (!rcc_get_ppre2()) {
        /* without APB2 prescaler, runs at APB2 freq */
        return rcc_apb2_frequency;
      } else {
        /* with any ABP2 prescaler, runs at 2 * APB2 freq */
        return rcc_apb2_frequency * 2;
      }

      // timers on low speed APB1
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
      if (!rcc_get_ppre1()) {
        /* without APB1 prescaler, runs at APB1 freq */
        return rcc_apb1_frequency;
      } else {
        /* with any ABP1 prescaler, runs at 2 * APB1 freq */
        return rcc_apb1_frequency * 2;
      }
    default:
      // other timers currently not supported
      break;
  }
  return 0;
}
