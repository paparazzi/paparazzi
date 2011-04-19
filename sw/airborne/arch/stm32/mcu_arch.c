/*
 * Paparazzi stm32 arch dependant microcontroller initialisation function
 *
 * Copyright (C) 2010 The Paparazzi team
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

#include "mcu.h"

#include BOARD_CONFIG

#include <inttypes.h>
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#ifdef USE_OPENCM3
#include <libopencm3/stm32/rcc.h>
#endif


void mcu_arch_init(void) {
#ifdef USE_OPENCM3
  rcc_clock_setup_in_hse_12mhz_out_72mhz();
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
  return;
#endif
#ifdef HSE_TYPE_EXT_CLK
#warning Using external clock
  /* Setup the microcontroller system.
   *  Initialize the Embedded Flash Interface,
   *  initialize the PLL and update the SystemFrequency variable.
   */
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();
  /* Enable HSE with external clock ( HSE_Bypass ) */
  RCC_HSEConfig( STM32_RCC_MODE );
  /* Wait till HSE is ready */
  ErrorStatus HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if (HSEStartUpStatus != SUCCESS) {
    /* block if something went wrong */
    while(1) {}
  }
  else {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, STM32_PLL_MULT);
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08) {}
  }
#else  /* HSE_TYPE_EXT_CLK */
#warning Using normal system clock setup
  SystemInit();
#endif /* HSE_TYPE_EXT_CLK */
   /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

#ifdef STM32_FORCE_ALL_CLOCK_ON
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
#endif  


}

