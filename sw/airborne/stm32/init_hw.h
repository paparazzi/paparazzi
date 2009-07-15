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
#include CONFIG
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/flash.h>
#include <stm32/misc.h>


#ifdef PERIPHERALS_AUTO_INIT
#ifdef USE_LED
#include "led.h"
#endif
#if defined USE_UART1 || defined USE_UART2 || defined USE_UART3
#include "uart.h"
#endif
#if defined USE_I2C0 || defined USE_I2C1
#include "i2c.h"
#endif
#endif /* PERIPHERALS_AUTO_INIT */


/* declare functions and values from crt0.S & the linker control file */
extern void reset(void);
/* extern void exit(void); */
extern void abort(void);


static inline void hw_init(void) {
/* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
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
#ifdef USE_I2C0
  i2c0_init();
#endif
#ifdef USE_I2C1
  i2c1_init();
#endif
#endif /* PERIPHERALS_AUTO_INIT */


}

#endif /* INIT_HW_H */
