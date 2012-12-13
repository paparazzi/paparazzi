/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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
#include "peripherals/max1168.h"

#include "libopencm3/stm32/f1/rcc.h"
#include "libopencm3/stm32/f1/gpio.h"
#include "libopencm3/stm32/exti.h"
#include "libopencm3/stm32/f1/nvic.h"

void max1168_arch_init( void ) {

  /* configure external interrupt exti2 on PD2( data ready ) v1.0*/
  /*                                       PB2( data ready ) v1.1*/
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT, GPIO2);

  exti_select_source(EXTI2, GPIOB);
  exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI2);

  nvic_set_priority(NVIC_EXTI2_IRQ, 0xF);
  nvic_enable_irq(NVIC_EXTI2_IRQ);

}

void exti2_isr(void) {

  /* clear EXTI */
  exti_reset_request(EXTI2);

  max1168_status = MAX1168_GOT_EOC;

}

