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

#ifndef LED_HW_H
#define LED_HW_H

#include <stm32/gpio.h>
#include <stm32/rcc.h>

#include BOARD_CONFIG

#include "std.h"


/*
 *
 *  Regular GPIO driven LEDs
 *
 */
#ifndef LED_STP08

#define _LED_GPIO_CLK(i)  i
#define _LED_GPIO(i)  i
#define _LED_GPIO_PIN(i) i
#define _LED_AFIO_REMAP(i) i

#define LED_GPIO_CLK(i) _LED_GPIO_CLK(LED_ ## i ## _GPIO_CLK)
#define LED_GPIO(i) _LED_GPIO(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_GPIO_PIN(LED_ ## i ## _GPIO_PIN)
#define LED_AFIO_REMAP(i) _LED_AFIO_REMAP(LED_ ## i ## _AFIO_REMAP)

/* set pin as output */
#define LED_INIT(i) {					\
    GPIO_InitTypeDef GPIO_InitStructure; 		\
    RCC_APB2PeriphClockCmd(LED_GPIO_CLK(i), ENABLE);	\
    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN(i);	\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	\
    GPIO_Init(LED_GPIO(i), &GPIO_InitStructure);	\
    LED_AFIO_REMAP(i);					\
  }

#define LED_ON(i) {LED_GPIO(i)->BSRR = LED_GPIO_PIN(i);}
#define LED_OFF(i) { LED_GPIO(i)->BRR  = LED_GPIO_PIN(i);}
#define LED_TOGGLE(i) {	LED_GPIO(i)->ODR ^= LED_GPIO_PIN(i);}

#define LED_PERIODIC() {}


/*
 *
 * Shift register driven LEDs
 *
 */
#else  /* LED_STP08 */
#define NB_LED 8
extern uint8_t led_status[NB_LED];
/* Lisa/L uses a shift register for driving LEDs */
/* PA8  led_clk  */
/* PC15 led_data */

#define LED_INIT(_i) {						\
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	\
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	\
    GPIO_InitTypeDef GPIO_InitStructure;			\
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;			\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		\
    GPIO_Init(GPIOA, &GPIO_InitStructure);			\
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;			\
    GPIO_Init(GPIOC, &GPIO_InitStructure);			\
    for(uint8_t i=0; i<NB_LED; i++)				\
      led_status[i] = FALSE;					\
  }

#define LED_ON(i)  { led_status[i] = TRUE;  }
#define LED_OFF(i) { led_status[i] = FALSE; }
#define LED_TOGGLE(i) {led_status[i] = !led_status[i];}

#define LED_PERIODIC() {					\
    for (uint8_t cnt = 0; cnt < NB_LED; cnt++) {		\
      if (led_status[cnt])					\
	GPIOC->BSRR = GPIO_Pin_15;				\
      else							\
	GPIOC->BRR = GPIO_Pin_15;				\
      GPIOA->BSRR = GPIO_Pin_8; /* clock rising edge */		\
      GPIOA->BRR = GPIO_Pin_8;  /* clock falling edge */	\
    }								\
  }

#endif /* LED_STP08 */

#endif /* LED_HW_H */
