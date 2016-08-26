/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @brief chibios arch dependant gpio functions
 * @details In Chibios palSet/Clear/Toggle(port, pin)
 * 		    replaces gpio functions
 *
 */
#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

/**
 * Set a gpio output to high level.
 */
static inline void gpio_output_high(uint32_t port, uint16_t pin) {
  palSetPad(port, pin);
}

/**
 * Clear a gpio output to low level.
 */
static inline void gpio_output_low(uint32_t port, uint16_t pin) {
  palClearPad(port, pin);
}

#endif /* GPIO_ARCH_H */
