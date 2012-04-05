/*
 * Copyright (C) 2010 Eric Parsonage
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
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

/** @file battery_buzzer.c
 * Simple module to toggle a gpio with connected buzzer on Lisa/M,
 * when battery voltage drops below a certain level.
 */

#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include "battery_buzzer.h"
#include "generated/airframe.h"
#include "subsystems/electrical.h"

/* initialises GPIO pins */
void battery_buzzer_init(void) {
  /* initialise peripheral clock for port C */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* set port C pin 10 to be low */
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);

  battery_buzzer_periodic();
}

/* sets GPIO pins */
void battery_buzzer_periodic(void) {

  if (electrical.vsupply < (LOW_BAT_LEVEL * 10)) {
    GPIO_WriteBit(GPIOC, GPIO_Pin_10 , Bit_RESET);
  } else {
    GPIO_WriteBit(GPIOC, GPIO_Pin_10 , Bit_SET);
  }
}
