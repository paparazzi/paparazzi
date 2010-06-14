/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

/*
 *\brief STM low level hardware initialisation 
 * PLL, MAM, VIC
 *
 */

#ifndef INIT_HW_H
#define INIT_HW_H

#include <inttypes.h>
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/flash.h>
#include <stm32/misc.h>

#include BOARD_CONFIG

#ifdef PERIPHERALS_AUTO_INIT
#ifdef USE_LED
#include "led.h"
#endif
#if defined USE_UART1 || defined USE_UART2 || defined USE_UART3
#include "uart.h"
#endif
#if defined USE_I2C1 || defined USE_I2C2
#include "i2c.h"
#endif
#endif /* PERIPHERALS_AUTO_INIT */


/* declare functions and values from crt0.S & the linker control file */
extern void reset(void);
/* extern void exit(void); */
extern void abort(void);


static inline void hw_init(void) {

#ifdef HSE_TYPE_EXT_CLK
  /* Setup the microcontroller system. 
   *  Initialize the Embedded Flash Interface,  
   *  initialize the PLL and update the SystemFrequency variable. 
   */
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();
  /* Enable HSE with external clock ( HSE_Bypass ) */
  RCC_HSEConfig( RCC_HSE_Bypass );
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
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
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
  SystemInit();
#endif /* HSE_TYPE_EXT_CLK */
   /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);

#if 0
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, DISABLE);  
#endif

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
			 RCC_APB2Periph_GPIOB |
			 RCC_APB2Periph_GPIOC |
			 RCC_APB2Periph_GPIOD |
			 RCC_APB2Periph_AFIO, ENABLE);
  



#ifdef PERIPHERALS_AUTO_INIT
#ifdef USE_LED
  led_init();
#endif
#ifdef USE_UART1
  uart1_init();
#endif
#ifdef USE_UART2
  uart2_init();
#endif
#ifdef USE_UART3
  uart3_init();
#endif
#ifdef USE_I2C1
  i2c1_init();
#endif
#ifdef USE_I2C2
  i2c2_init();
#endif
#endif /* PERIPHERALS_AUTO_INIT */


}

#endif /* INIT_HW_H */
