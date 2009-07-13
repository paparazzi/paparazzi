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

#include CONFIG
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include "std.h"

#define _LED_GPIO_CLK(i)  i
#define _LED_GPIO(i)  i
#define _LED_GPIO_PIN(i) i

#define LED_GPIO_CLK(i) _LED_GPIO_CLK(LED_ ## i ## _GPIO_CLK)
#define LED_GPIO(i) _LED_GPIO(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_GPIO_PIN(LED_ ## i ## _GPIO_PIN)

/* set pin as output */
#define LED_INIT(i) {					\
    GPIO_InitTypeDef GPIO_InitStructure; 		\
    RCC_APB2PeriphClockCmd(LED_GPIO_CLK(i), ENABLE);	\
    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN(i);	\
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	\
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	\
    GPIO_Init(LED_GPIO(i), &GPIO_InitStructure);	\
  }

#define LED_ON(i) { LED_GPIO(i)->BRR  = LED_GPIO_PIN(i);}
#define LED_OFF(i) {LED_GPIO(i)->BSRR = LED_GPIO_PIN(i);}
#define LED_TOGGLE(i) {	LED_GPIO(i)->ODR ^= LED_GPIO_PIN(i);}



#endif /* LED_HW_H */
