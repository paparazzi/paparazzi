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
 *
 * The gpio_set and gpio_clear functions are already available from libopencm3.
 */

#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

#include <libopencm3/stm32/gpio.h>

/**
 * Abstract gpio port type for hardware independent part
 */
typedef uint32_t gpio_port_t;

/**
 * Setup one or more pins of the given GPIO port as outputs.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
extern void gpio_setup_output(uint32_t port, uint16_t gpios);

/**
 * Setup one or more pins of the given GPIO port as inputs.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
extern void gpio_setup_input(uint32_t port, uint16_t gpios);

/**
 * Setup one or more pins of the given GPIO port as inputs with pull up resistor enabled.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
extern void gpio_setup_input_pullup(uint32_t port, uint16_t gpios);

/**
 * Setup one or more pins of the given GPIO port as inputs with pull down resistors enabled.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
extern void gpio_setup_input_pulldown(uint32_t port, uint16_t gpios);

/**
 * Setup a gpio for input or output with alternate function.
 * This is an STM32 specific helper funtion and should only be used in stm32 arch code.
 */
#if defined(STM32F1)
extern void gpio_setup_pin_af(uint32_t port, uint16_t pin, uint32_t af, bool is_output);
#else
extern void gpio_setup_pin_af(uint32_t port, uint16_t pin, uint8_t af, bool is_output);
#endif

/**
 * Setup a gpio for analog use.
 * This is an STM32 specific helper funtion and should only be used in stm32 arch code.
 */
extern void gpio_setup_pin_analog(uint32_t port, uint16_t pin);

/**
 * Enable the relevant clock.
 * This is an STM32 specific helper funtion and should only be used in stm32 arch code.
 */
extern void gpio_enable_clock(uint32_t port);

#endif /* GPIO_ARCH_H */
