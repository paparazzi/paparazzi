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

#include "gpio_ext_common.h"

#include "generated/airframe.h"
#include "mcu_periph/gpio.h"
#include "led.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>


/* External GPIO providers */
#include "gpio_ext_pca95xx.h"

#define GPIO_EXT_NOT_PROVIDED NULL
#define GPIO_EXT_PCA95XX &pca95xx_functions


#ifndef GPIO_EXT_PROVIDER1
#define GPIO_EXT_PROVIDER1 GPIO_EXT_NOT_PROVIDED
#endif
#ifndef GPIO_EXT_PROVIDER2
#define GPIO_EXT_PROVIDER2 GPIO_EXT_NOT_PROVIDED
#endif
#ifndef GPIO_EXT_PROVIDER3
#define GPIO_EXT_PROVIDER3 GPIO_EXT_NOT_PROVIDED
#endif
#ifndef GPIO_EXT_PROVIDER4
#define GPIO_EXT_PROVIDER4 GPIO_EXT_NOT_PROVIDED
#endif

static struct gpio_ext_functions *gpio_ext_impl[GPIOEXT_NB] = {
    GPIO_EXT_PROVIDER1,
    GPIO_EXT_PROVIDER2,
    GPIO_EXT_PROVIDER3,
    GPIO_EXT_PROVIDER4,
};


/* External GPIO configuration */
// Not all defines need to be used by all implementations
#ifndef GPIO_EXT_BLOCKING1
#define GPIO_EXT_BLOCKING1 TRUE
#endif
#ifndef GPIO_EXT_BLOCKING2
#define GPIO_EXT_BLOCKING2 TRUE
#endif
#ifndef GPIO_EXT_BLOCKING3
#define GPIO_EXT_BLOCKING3 TRUE
#endif
#ifndef GPIO_EXT_BLOCKING4
#define GPIO_EXT_BLOCKING4 TRUE
#endif
const bool gpio_ext_blocking[] = {
    GPIO_EXT_BLOCKING1,
    GPIO_EXT_BLOCKING1,
    GPIO_EXT_BLOCKING1,
    GPIO_EXT_BLOCKING1,
};

#ifndef GPIO_EXT_I2C_PERIPH1
#define GPIO_EXT_I2C_PERIPH1 NULL
#endif
#ifndef GPIO_EXT_I2C_PERIPH2
#define GPIO_EXT_I2C_PERIPH2 NULL
#endif
#ifndef GPIO_EXT_I2C_PERIPH3
#define GPIO_EXT_I2C_PERIPH3 NULL
#endif
#ifndef GPIO_EXT_I2C_PERIPH4
#define GPIO_EXT_I2C_PERIPH4 NULL
#endif
struct i2c_periph *gpio_ext_i2c_periph[] = {
    GPIO_EXT_I2C_PERIPH1,
    GPIO_EXT_I2C_PERIPH2,
    GPIO_EXT_I2C_PERIPH3,
    GPIO_EXT_I2C_PERIPH4,
};

#ifndef GPIO_EXT_I2C_ADDRESS1
#define GPIO_EXT_I2C_ADDRESS1 0x00
#endif
#ifndef GPIO_EXT_I2C_ADDRESS2
#define GPIO_EXT_I2C_ADDRESS2 0x00
#endif
#ifndef GPIO_EXT_I2C_ADDRESS3
#define GPIO_EXT_I2C_ADDRESS3 0x00
#endif
#ifndef GPIO_EXT_I2C_ADDRESS4
#define GPIO_EXT_I2C_ADDRESS4 0x00
#endif
const uint8_t gpio_ext_i2c_addr[] = {
    GPIO_EXT_I2C_ADDRESS1,
    GPIO_EXT_I2C_ADDRESS2,
    GPIO_EXT_I2C_ADDRESS3,
    GPIO_EXT_I2C_ADDRESS4,
};



/*
 * External GPIO implementation
 */
#define SAFE_CALL(_port, _fn, _args...) if(gpio_ext_impl[_port - GPIOEXT1] && gpio_ext_impl[_port - GPIOEXT1]->_fn) gpio_ext_impl[_port - GPIOEXT1]->_fn(_args);

// Wrapping functions
void __wrap_gpio_setup_output(uint32_t port, uint32_t gpios);
void __real_gpio_setup_output(uint32_t port, uint32_t gpios);
void __wrap_gpio_setup_output(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(port, setup_output, port, gpios);
  } else {
    __real_gpio_setup_output(port, gpios);
  }
}

void __wrap_gpio_setup_input(uint32_t port, uint32_t gpios);
void __real_gpio_setup_input(uint32_t port, uint32_t gpios);
void __wrap_gpio_setup_input(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(port, setup_input, port, gpios);
  } else {
    __real_gpio_setup_input(port, gpios);
  }
}

uint32_t __wrap_gpio_get(uint32_t port, uint32_t gpios);
uint32_t __real_gpio_get(uint32_t port, uint32_t gpios);
uint32_t __wrap_gpio_get(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    if (gpio_ext_impl[port - GPIOEXT1] && gpio_ext_impl[port - GPIOEXT1]->get) {
      return gpio_ext_impl[port - GPIOEXT1]->get(port, gpios);
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
    SAFE_CALL(port, set, port, gpios);
  } else {
    __real_gpio_set(port, gpios);
  }
}

void __wrap_gpio_clear(uint32_t port, uint32_t gpios);
void __real_gpio_clear(uint32_t port, uint32_t gpios);
void __wrap_gpio_clear(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(port, clear, port, gpios);
  } else {
    __real_gpio_clear(port, gpios);
  }
}

void __wrap_gpio_toggle(uint32_t port, uint32_t gpios);
void __real_gpio_toggle(uint32_t port, uint32_t gpios);
void __wrap_gpio_toggle(uint32_t port, uint32_t gpios) {
  if (port >= GPIOEXT1 && port < GPIOEXT1 + GPIOEXT_NB) {
    SAFE_CALL(port, toggle, port, gpios);
  } else {
    __real_gpio_toggle(port, gpios);
  }
}


