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

/**
 * @file arch/stm32/peripherals/ms2100_arch.c
 *
 * STM32 specific functions for the ms2100 magnetic sensor from PNI.
 */

#include "peripherals/ms2100.h"
#include "mcu_periph/sys_time.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#ifndef STM32F1
#error "MS2100 arch currently only implemented for STM32F1"
#endif

void ms2100_arch_init(void)
{

  /* set mag reset as output (reset on PC13) ----*/
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_AFIO);
  gpio_set(GPIOC, GPIO13);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  Ms2100Reset();

  /* configure data ready input on PB5 */
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5);

  /* external interrupt for drdy pin */
  exti_select_source(EXTI5, GPIOB);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI5);

  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0f);
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

void ms2100_reset_cb(struct spi_transaction *t __attribute__((unused)))
{
  // set RESET pin high for at least 100 nsec
  // busy wait should not harm
  Ms2100Set();

  // FIXME, make nanosleep funcion
  uint32_t dt_ticks = cpu_ticks_of_nsec(110);
  int32_t end_cpu_ticks = systick_get_value() - dt_ticks;
  if (end_cpu_ticks < 0) {
    end_cpu_ticks += systick_get_reload();
  }
  while (systick_get_value() > (uint32_t)end_cpu_ticks)
    ;

  Ms2100Reset();
}

void exti9_5_isr(void)
{
  ms2100.status = MS2100_GOT_EOC;
  exti_reset_request(EXTI5);
}
