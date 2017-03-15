/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file arch/linux/mcu_periph/gpio_arch.c
 *
 * GPIO helper functions for linux/omap.
 * @todo implement gpio_set|clear
 */

#include "arch/linux/mcu_periph/gpio_arch.h"

void gpio_setup_output(uint32_t port, uint16_t gpios) {}

void gpio_setup_input(uint32_t port, uint16_t gpios) {}

void gpio_set(uint32_t port, uint16_t pin) {}

void gpio_clear(uint32_t port, uint16_t pin) {}

uint16_t gpio_get(uint32_t gpioport, uint16_t gpios) { return 0; }


