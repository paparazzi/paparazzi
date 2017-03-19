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
 * @file arch/chibios/led_hw.h
 * Led macro implementation for ChibiOS arch
 */
#ifndef LED_HW_H
#define LED_HW_H

/*
 * hal.h is needed for palXXX functions
 */
#include <hal.h>
#include "mcu_periph/gpio_def.h"
#include BOARD_CONFIG

/*
 * Regular GPIO driven LEDs
 */
#define _LED_GPIO(i)  i
#define _LED_GPIO_PIN(i) i

#define LED_GPIO(i) _LED_GPIO(LED_ ## i ## _GPIO)
#define LED_GPIO_PIN(i) _LED_GPIO_PIN(LED_ ## i ## _GPIO_PIN)

#define LED_INIT(i) palSetPadMode(LED_GPIO(i), LED_GPIO_PIN(i), PAL_MODE_OUTPUT_PUSHPULL)
#define LED_ON(i) palClearPad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_OFF(i) palSetPad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_TOGGLE(i) palTogglePad(LED_GPIO(i), LED_GPIO_PIN(i))
#define LED_DISABLE(i) palSetPadMode(LED_GPIO(i), LED_GPIO_PIN(i), PAL_MODE_INPUT)
#define LED_PERIODIC() {}

#endif /* LED_HW_H */
