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
 * @file arch/linux/mcu_periph/gpio_arch.h
 *
 * GPIO helper functions for linux/omap.
 * @todo implement pprz_gpio_set|clear
 */

#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

#include "std.h"
#include <stdint.h>

typedef uint32_t gpio_port_t;



/**
 * Read a gpio value.
 */
extern uint16_t gpio_get(gpio_port_t gpioport, uint16_t gpios);

#endif /* GPIO_ARCH_H */
