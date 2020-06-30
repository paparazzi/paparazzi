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
 * @file arch/lpc21/mcu_periph/gpio_arch.h
 *
 * GPIO helper functions for LPC21xx.
 */

#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

#include "std.h"
#include "LPC21xx.h"

typedef uint32_t gpio_port_t;

#define GPIOA    0
#define GPIOB    1

#define GPIO0    (1 << 0)
#define GPIO1    (1 << 1)
#define GPIO2    (1 << 2)
#define GPIO3    (1 << 3)
#define GPIO4    (1 << 4)
#define GPIO5    (1 << 5)
#define GPIO6    (1 << 6)
#define GPIO7    (1 << 7)
#define GPIO8    (1 << 8)
#define GPIO9    (1 << 9)
#define GPIO10   (1 << 10)
#define GPIO11   (1 << 11)
#define GPIO12   (1 << 12)
#define GPIO13   (1 << 13)
#define GPIO14   (1 << 14)
#define GPIO15   (1 << 15)
#define GPIO16   (1 << 16)
#define GPIO17   (1 << 17)
#define GPIO18   (1 << 18)
#define GPIO19   (1 << 19)
#define GPIO20   (1 << 20)
#define GPIO21   (1 << 21)
#define GPIO22   (1 << 22)
#define GPIO23   (1 << 23)
#define GPIO24   (1 << 24)
#define GPIO25   (1 << 25)
#define GPIO26   (1 << 26)
#define GPIO27   (1 << 27)
#define GPIO28   (1 << 28)
#define GPIO29   (1 << 29)
#define GPIO30   (1 << 30)
#define GPIO31   (1 << 31)


/**
 * Setup one or more pins of the given GPIO port as outputs.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
static inline void gpio_setup_output(gpio_port_t port, uint32_t gpios)
{
  if (port == 0) {
    IO0DIR |= gpios;
  } else if (port == 1) {
    IO1DIR |= gpios;
  }
}

/**
 * Setup one or more pins of the given GPIO port as inputs.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
static inline void gpio_setup_input(gpio_port_t port, uint32_t gpios)
{
  if (port == 0) {
    IO0DIR &= ~gpios;
  } else if (port == 1) {
    IO1DIR &= ~gpios;
  }
}

/**
 * Set one or more pins of the given GPIO port to high level.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
static inline void gpio_set(gpio_port_t port, uint32_t gpios)
{
  if (port == 0) {
    IO0SET = gpios;
  } else if (port == 1) {
    IO1SET = gpios;
  }
}

/**
 * Clear one or more pins of the given GPIO port to low level.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
static inline void gpio_clear(gpio_port_t port, uint32_t gpios)
{
  if (port == 0) {
    IO0CLR = gpios;
  } else if (port == 1) {
    IO1CLR = gpios;
  }
}

/**
 * Toggle a one or more pins of the given GPIO port.
 * @param[in] port
 * @param[in] gpios If multiple pins are to be changed, use logical OR '|' to separate them.
 */
static inline void gpio_toggle(gpio_port_t port, uint32_t gpios)
{
  if (port == 0) {
    uint32_t set_gpios = IO0PIN;
    // clear selected gpio pins which are currently set
    IO0CLR = set_gpios & gpios;
    // set selected gpio pins which are currently cleared
    IO0SET = ~set_gpios & gpios;
  } else if (port == 1) {
    uint32_t set_gpios = IO1PIN;
    // clear selected gpio pins which are currently set
    IO1CLR = set_gpios & gpios;
    // set selected gpio pins which are currently cleared
    IO1SET = ~set_gpios & gpios;
  }
}

/**
 * Read the value of one or more pins of the given GPIO port.
 * @param[in] port  GPIO port (0 or 1)
 * @param[in] gpios GPIO pin(s). If multiple pins are to be changed, use logical OR '|' to separate them.
 */
static inline uint32_t gpio_get(gpio_port_t port, uint32_t gpios)
{
  if (port == 0) {
    return IO0PIN & gpios;
  } else if (port == 1) {
    return IO1PIN & gpios;
  }
}

#endif /* GPIO_ARCH_H */
