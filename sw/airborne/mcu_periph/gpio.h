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
 * @file mcu_periph/gpio.h
 *
 * Some architecture independent helper functions for GPIOs.
 *
 * Functions which should be supported by each architecture implementation:
 * - pprz_gpio_setup_output(port, gpios)
 * - pprz_gpio_setup_input(port, gpios)
 * - gpio_get(port, gpios)
 * - pprz_gpio_set(port, gpios)
 * - pprz_gpio_clear(port, gpios)
 * - gpio_toggle(port, gpios)
 *
 * This includes the architecture specific header where the actual functions are declared.
 */

#ifndef MCU_PERIPH_GPIO_H
#define MCU_PERIPH_GPIO_H

#include "std.h"
#include "mcu_periph/gpio_arch.h"


/**
 * Setup one or more pins of the given GPIO port as outputs.
 * @param[in] port
 * @param[in] gpios
 */
void pprz_gpio_setup_output(gpio_port_t port, uint16_t gpios);

/**
 * Setup one or more pins of the given GPIO port as inputs.
 * @param[in] port
 * @param[in] gpios
 */
void pprz_gpio_setup_input(gpio_port_t port, uint16_t gpios);


/**
 * Set a gpio output to high level.
 */
void pprz_gpio_set(gpio_port_t port, uint16_t pin);

/**
 * Clear a gpio output to low level.
 */
void pprz_gpio_clear(gpio_port_t port, uint16_t pin);



#endif /* MCU_PERIPH_GPIO_H */
