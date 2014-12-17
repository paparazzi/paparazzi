/*
 * Copyright (C) 2010 Eric Parsonage
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
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include "deploy_sonar_buoy.h"
#include "generated/airframe.h"

/* simple module to toggle two gpio pins on Lisa.
 * The application in this was written for drops
 * two sonar buoys. TODO extend to a generalised
 * GPIO  module
 */

bool_t buoy_1;
bool_t buoy_2;

/* initialises GPIO pins */
void deploy_sonar_buoy_init(void)
{
  /* initialise peripheral clock for port C */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* set port C pin 5 to be low */
  GPIO_WriteBit(GPIOC, GPIO_Pin_5 , Bit_RESET);

  /* initialise peripheral clock for port B */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* set port B pin 0 to be low */
  GPIO_WriteBit(GPIOB, GPIO_Pin_0 , Bit_RESET);

  /* set the variables of interest to be FALSE */
  buoy_1 = FALSE;
  buoy_2 = FALSE;
  deploy_sonar_buoy_periodic();
}

/* sets GPIO pins */
void deploy_sonar_buoy_periodic(void)
{
  GPIO_WriteBit(GPIOC, GPIO_Pin_5 , buoy_1 ? Bit_SET : Bit_RESET);
  GPIO_WriteBit(GPIOB, GPIO_Pin_0 , buoy_2 ? Bit_SET : Bit_RESET);
}
