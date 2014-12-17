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
#include "peripherals/hmc5843.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/i2c.h"

#ifndef STM32F1
#error "HMC5843 arch currently only implemented for STM32F1"
#endif

void hmc5843_arch_init(void)
{
  /* configure external interrupt exti5 on PB5( mag int ) */
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_AFIO);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT, GPIO5);

#ifdef HMC5843_USE_INT
  exti_select_source(EXTI5, GPIOB);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI5);

  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0f);
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
#endif
}

void hmc5843_arch_reset(void)
{
  i2c2_er_irq_handler();
}

void exti9_5_isr(void)
{

  exti_reset_request(EXTI5);
}
