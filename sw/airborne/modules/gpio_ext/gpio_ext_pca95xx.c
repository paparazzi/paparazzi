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

/** @file "modules/gpio_ext/gpio_ext_pca95xx.c"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * PCA95XX external GPIO peripheral
 */

#include "modules/gpio_ext/gpio_ext_pca95xx.h"

#include "generated/airframe.h"

#include "modules/gpio_ext/gpio_ext_common.h"
#include "peripherals/pca95xx.h"
#include "mcu_periph/i2c.h"

#include <stdbool.h>
#include <assert.h>


static void gpio_ext_pca95xx_setup_output(uint32_t port, uint32_t gpios);
static void gpio_ext_pca95xx_setup_input(uint32_t port, uint32_t gpios);
static uint32_t gpio_ext_pca95xx_get(uint32_t port, uint32_t gpios);
static void gpio_ext_pca95xx_set(uint32_t port, uint32_t gpios);
static void gpio_ext_pca95xx_clear(uint32_t port, uint32_t gpios);
static void gpio_ext_pca95xx_toggle(uint32_t port, uint32_t gpios);


struct gpio_ext_functions pca95xx_functions = {
    gpio_ext_pca95xx_setup_output,
    gpio_ext_pca95xx_setup_input,
    gpio_ext_pca95xx_get,
    gpio_ext_pca95xx_set,
    gpio_ext_pca95xx_clear,
    gpio_ext_pca95xx_toggle,
};


struct gpio_ext_pca95xx_impl_t {
  struct pca95xx periph;
  uint8_t output_reg;
  uint8_t config_reg;
  bool initialized;
};
static struct gpio_ext_pca95xx_impl_t impl[GPIOEXT_NB]; // Initialized 0, so .initialized = false!


static void gpio_ext_pca95xx_lazy_init(uint32_t port) {
  int i = port - GPIOEXT1;
  if (impl[i].initialized) return;
  /* Set up pca95xx implementation struct */
  impl[i].output_reg = 0xFF;
  impl[i].config_reg = 0xFF;
  /* Set up pca95xx peripheral */
  pca95xx_init(&impl[i].periph, gpio_ext_i2c_periph[i], gpio_ext_i2c_addr[i]);
  /* Configure pins as input (default) and wait for IC to wake up */
  do {
    pca95xx_configure(&impl[i].periph, 0xFF, true);
  } while (impl[i].periph.i2c_trans.status != I2CTransSuccess);
  /* Mark initialization complete */
  impl[i].initialized = true;
}


static void gpio_ext_pca95xx_setup_output(uint32_t port, uint32_t gpios) {
  gpio_ext_pca95xx_lazy_init(port);
  int i = port - GPIOEXT1;
  impl[i].config_reg &= ~gpios;
  pca95xx_configure(&impl[i].periph, impl[i].config_reg, gpio_ext_blocking[i]);
}

static void gpio_ext_pca95xx_setup_input(uint32_t port, uint32_t gpios) {
  gpio_ext_pca95xx_lazy_init(port);
  int i = port - GPIOEXT1;
  impl[i].config_reg |= gpios;
  pca95xx_configure(&impl[i].periph, impl[i].config_reg, gpio_ext_blocking[i]);
}

static uint32_t gpio_ext_pca95xx_get(uint32_t port, uint32_t gpios) {
  int i = port - GPIOEXT1;
  uint8_t result;
  pca95xx_get_input(&impl[i].periph, gpios, &result);
  return result;
}

static void gpio_ext_pca95xx_set(uint32_t port, uint32_t gpios) {
  int i = port - GPIOEXT1;
  impl[i].output_reg |= gpios;
  pca95xx_set_output(&impl[i].periph, impl[i].output_reg, gpio_ext_blocking[i]);
}

static void gpio_ext_pca95xx_clear(uint32_t port, uint32_t gpios) {
  int i = port - GPIOEXT1;
  impl[i].output_reg &= ~gpios;
  pca95xx_set_output(&impl[i].periph, impl[i].output_reg, gpio_ext_blocking[i]);
}

static void gpio_ext_pca95xx_toggle(uint32_t port, uint32_t gpios) {
  int i = port - GPIOEXT1;
  impl[i].output_reg ^= gpios;
  pca95xx_set_output(&impl[i].periph, impl[i].output_reg, gpio_ext_blocking[i]);
}





