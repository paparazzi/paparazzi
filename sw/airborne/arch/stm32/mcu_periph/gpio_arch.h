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
 * @file arch/stm32/mcu_periph/gpio_arch.h
 * @ingroup stm32_arch
 *
 * GPIO helper functions for STM32F1 and STM32F4.
 */

#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

#include <libopencm3/stm32/gpio.h>

/**
 * Set a gpio output to high level.
 */
static inline void gpio_output_high(uint32_t port, uint16_t pin) {
  gpio_set(port, pin);
}

/**
 * Clear a gpio output to low level.
 */
static inline void gpio_output_low(uint32_t port, uint16_t pin) {
  gpio_clear(port, pin);
}

/**
 * Setup a gpio for input or output with alternate function.
 */
extern void gpio_setup_pin_af(uint32_t port, uint16_t pin, uint8_t af, bool_t is_output);

extern void gpio_enable_clock(uint32_t port);

#endif /* GPIO_ARCH_H */
