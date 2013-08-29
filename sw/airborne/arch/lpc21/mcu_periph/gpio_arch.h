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

/**
 * Set a gpio output to high level.
 */
static inline void gpio_set(uint32_t port, uint16_t pin) {
  if (port == 0)
    IO0SET = _BV(pin);
  else if (port == 1)
    IO1SET = _BV(pin);
}

/**
 * Clear a gpio output to low level.
 */
static inline void gpio_clear(uint32_t port, uint16_t pin) {
  if (port == 0)
    IO0CLR = _BV(pin);
  else if (port == 1)
    IO1CLR = _BV(pin);
}


#endif /* GPIO_ARCH_H */
