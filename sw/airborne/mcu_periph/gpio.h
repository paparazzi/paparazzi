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
 */

#ifndef MCU_PERIPH_GPIO_H
#define MCU_PERIPH_GPIO_H

#include "std.h"
#include "mcu_periph/gpio_arch.h"

/**
 * Setup gpio pin as generic output.
 */
extern void gpio_setup_output(uint32_t port, uint16_t pin);

/**
 * Setup a gpio pin as generic input.
 */
extern void gpio_setup_input(uint32_t port, uint16_t pin);

#endif /* MCU_PERIPH_GPIO_H */
