/*
 * Copyright (C) Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/gpio_ext/gpio_ext_common.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Common external GPIO functions.
 */

#include "modules/gpio_ext/gpio_ext_common.h"

#include "mcu_periph/gpio.h"
#include "led.h"

#include <stdint.h>
#include <assert.h>


#define SAFE_CALL(_fn, _args...) if (_fn) _fn(_args);


// Function pointer structs to external gpio implementations
struct gpio_periph_t gpio_ext[GPIOEXT_NB];  // All initialized NULL!


void gpio_ext_common_init(void)
{
  // your init code here
}

void gpio_ext_common_event(void)
{
  // your event code here
}

// Wrapping functions
void __wrap_gpio_setup_output(uint32_t port, uint32_t gpios);
void __real_gpio_setup_output(uint32_t port, uint32_t gpios);
void __wrap_gpio_setup_output(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(gpio_ext[port - GPIOEXT1].setup_output, port, gpios);
  } else {
    __real_gpio_setup_output(port, gpios);
  }
}

void __wrap_gpio_setup_input(uint32_t port, uint32_t gpios);
void __real_gpio_setup_input(uint32_t port, uint32_t gpios);
void __wrap_gpio_setup_input(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(gpio_ext[port - GPIOEXT1].setup_input, port, gpios);
  } else {
    __real_gpio_setup_input(port, gpios);
  }
}

uint32_t __wrap_gpio_get(uint32_t port, uint32_t gpios);
uint32_t __real_gpio_get(uint32_t port, uint32_t gpios);
uint32_t __wrap_gpio_get(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    if (gpio_ext[port - GPIOEXT1].get) {
      return gpio_ext[port - GPIOEXT1].get(port, gpios);
    } else {
      return 0;
    }
  } else {
    return __real_gpio_get(port, gpios);
  }
}

void __wrap_gpio_set(uint32_t port, uint32_t gpios);
void __real_gpio_set(uint32_t port, uint32_t gpios);
void __wrap_gpio_set(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(gpio_ext[port - GPIOEXT1].set, port, gpios);
  } else {
    __real_gpio_set(port, gpios);
  }
}

void __wrap_gpio_clear(uint32_t port, uint32_t gpios);
void __real_gpio_clear(uint32_t port, uint32_t gpios);
void __wrap_gpio_clear(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(gpio_ext[port - GPIOEXT1].clear, port, gpios);
  } else {
    __real_gpio_clear(port, gpios);
  }
}

void __wrap_gpio_toggle(uint32_t port, uint32_t gpios);
void __real_gpio_toggle(uint32_t port, uint32_t gpios);
void __wrap_gpio_toggle(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(gpio_ext[port - GPIOEXT1].toggle, port, gpios);
  } else {
    __real_gpio_toggle(port, gpios);
  }
}


