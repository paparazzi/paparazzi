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
 * @file arch/chibios/mcu_periph/gpio_arch.h
 * gpio functions implemented for ChibiOS arch
 */
#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

#include "hal.h"
#include "mcu_periph/gpio_def.h"

/**
 * Setup one or more pins of the given GPIO port as outputs.
 * @param[in] port
 * @param[in] gpios
 */
extern void gpio_setup_output(ioportid_t port, uint16_t gpios);

/**
 * Setup one or more pins of the given GPIO port as inputs.
 * @param[in] port
 * @param[in] gpios
 */
extern void gpio_setup_intput(ioportid_t port, uint16_t gpios);

/**
 * Setup one or more pins of the given GPIO port as inputs with pull up resistor enabled.
 * @param[in] port
 * @param[in] gpios
 */
extern void gpio_setup_input_pullup(ioportid_t port, uint16_t gpios);

/**
 * Setup one or more pins of the given GPIO port as inputs with pull down resistors enabled.
 * @param[in] port
 * @param[in] gpios
 */
extern void gpio_setup_input_pulldown(ioportid_t port, uint16_t gpios);

/**
 * Setup a gpio for input or output with alternate function.
 * This is an STM32 specific helper funtion and should only be used in stm32 code.
 */
extern void gpio_setup_pin_af(ioportid_t port, uint16_t pin, uint8_t af);

/**
 * Setup a gpio for analog use.
 * @param[in] port
 * @param[in] pin
 */
extern void gpio_setup_pin_analog(ioportid_t port, uint16_t pin);


/**
 * Macro: Set a gpio output to high level.
 * @param[in] port
 * @param[in] pin
 */
#define gpio_set(_port, _pin) palSetPad(_port, _pin)

/**
 * Macro: Clear a gpio output to low level.
 * @param[in] port
 * @param[in] pin
 */
#define gpio_clear(_port, _pin) palClearPad(_port, _pin)

/**
 * Macro: Toggle a gpio output to low level.
 * @param[in] port
 * @param[in] pin
 */
#define gpio_toggle(_port,_pin) palTogglePad(_port, _pin)

#endif /* GPIO_ARCH_H */
