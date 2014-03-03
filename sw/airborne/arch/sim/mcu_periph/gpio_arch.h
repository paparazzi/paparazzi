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
 * @file arch/sim/mcu_periph/gpio_arch.h
 *
 * GPIO dummy function/macros to compile in simulation
 *
 */

#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

#define GPIOA 0
#define GPIOB 0
#define GPIOC 0
#define GPIOD 0
#define GPIOE 0
#define GPIOF 0
#define GPIOG 0
#define GPIOH 0

#define GPIO0 0
#define GPIO1 0
#define GPIO2 0
#define GPIO3 0
#define GPIO4 0
#define GPIO5 0
#define GPIO6 0
#define GPIO7 0
#define GPIO8 0
#define GPIO9 0
#define GPIO10 0
#define GPIO11 0
#define GPIO12 0
#define GPIO13 0
#define GPIO14 0
#define GPIO15 0
// on LCP21xx we have 32bit wide ports
#define GPIO16 0
#define GPIO17 0
#define GPIO18 0
#define GPIO19 0
#define GPIO20 0
#define GPIO21 0
#define GPIO22 0
#define GPIO23 0
#define GPIO24 0
#define GPIO25 0
#define GPIO26 0
#define GPIO27 0
#define GPIO28 0
#define GPIO29 0
#define GPIO30 0
#define GPIO31 0

static inline void gpio_setup_output(uint32_t port __attribute__((unused)), uint16_t pin __attribute__((unused))) {}
static inline void gpio_setup_input(uint32_t port __attribute__((unused)), uint16_t pin __attribute__((unused))) {}
static inline void gpio_set(uint32_t port __attribute__((unused)), uint16_t pin __attribute__((unused))) {}
static inline void gpio_clear(uint32_t port __attribute__((unused)), uint16_t pin __attribute__((unused))) {}
static inline void gpio_toggle(uint32_t port __attribute__((unused)), uint16_t pin __attribute__((unused))) {}

#endif /* GPIO_ARCH_H */
