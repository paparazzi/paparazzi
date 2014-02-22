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

#define gpio_setup_pin_af(port, pin, af, is_output) {}
#define gpio_setup_pin_analog(port, pin) {}
#define gpio_enable_clock(port) {}
#define gpio_set(port, pin) {}
#define gpio_clear(port, pin) {}

#endif /* GPIO_ARCH_H */
