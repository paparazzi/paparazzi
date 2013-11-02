/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file arch/stm32/mcu_periph/gpio_arch.c
 * @ingroup stm32_arch
 *
 * GPIO helper functions for STM32F1 and STM32F4.
 */

#include "mcu_periph/gpio.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#ifdef STM32F1
void gpio_enable_clock(uint32_t port) {
  switch (port) {
    case GPIOA:
      rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
      break;
    case GPIOB:
      rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
      break;
    case GPIOC:
      rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
      break;
    case GPIOD:
      rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);
      break;
    default:
      break;
  };
}

void gpio_setup_output(uint32_t port, uint16_t pin) {
  gpio_enable_clock(port);
  gpio_set_mode(port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pin);
}

void gpio_setup_input(uint32_t port, uint16_t pin) {
  gpio_enable_clock(port);
  gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pin);
}

void gpio_setup_pin_af(uint32_t port, uint16_t pin, uint8_t af, bool_t is_output) {
  gpio_enable_clock(port);
  /* remap alternate function if needed */
  if (af) {
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
    AFIO_MAPR |= af;
  }
  if (is_output)
    gpio_set_mode(port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pin);
  else
    gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pin);
}

#elif defined STM32F4
void gpio_enable_clock(uint32_t port) {
  switch (port) {
    case GPIOA:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
      break;
    case GPIOB:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
      break;
    case GPIOC:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
      break;
    case GPIOD:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
      break;
    case GPIOE:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
      break;
    case GPIOF:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPFEN);
      break;
    case GPIOG:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPGEN);
      break;
    case GPIOH:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPHEN);
      break;
    case GPIOI:
      rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPIEN);
      break;
    default:
      break;
  };
}

void gpio_setup_output(uint32_t port, uint16_t pin) {
  gpio_enable_clock(port);
  gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
}

void gpio_setup_input(uint32_t port, uint16_t pin) {
  gpio_enable_clock(port);
  gpio_mode_setup(port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, pin);
}

void gpio_setup_pin_af(uint32_t port, uint16_t pin, uint8_t af, bool_t is_output __attribute__ ((unused))) {
  gpio_enable_clock(port);
  gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
  gpio_set_af(port, af, pin);
}
#endif

