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

struct gpio_ext_pca95xx_impl_t {
  struct pca95xx periph;
  uint8_t output_reg;
  uint8_t config_reg;
  bool blocking;
};
struct gpio_ext_pca95xx_impl_t gpio_ext_pca95xx_impl[GPIOEXT_NB];

static const uint32_t ports[] = { GPIO_EXT_PCA95XX_PORTS };
static const struct i2c_periph *i2c_periphs[] = { GPIO_EXT_PCA95XX_I2C_PERIPHS };
static const uint8_t i2c_addrs[] = { GPIO_EXT_PCA95XX_ADDRESSES };
static const bool blockings[] = { GPIO_EXT_PCA95XX_BLOCKINGS };


static void gpio_ext_pca95xx_setup_output(uint32_t port, uint32_t gpios) {

}

static void gpio_ext_pca95xx_setup_input(uint32_t port, uint32_t gpios) {

}

static uint32_t gpio_ext_pca95xx_get(uint32_t port, uint32_t gpios) {
  return 0;
}

static void gpio_ext_pca95xx_set(uint32_t port, uint32_t gpios) {

}

static void gpio_ext_pca95xx_clear(uint32_t port, uint32_t gpios) {

}

static void gpio_ext_pca95xx_toggle(uint32_t port, uint32_t gpios) {

}


void gpio_ext_pca95xx_init(void)
{
  int n = sizeof(ports) / sizeof(ports[0]);
  for (int i = 0; i < n; ++i) {
    int ext_i = ports[i] - GPIOEXT1;
    assert(gpio_ext[ext_i].impl_data == NULL);
    /* Set up gpio_periph struct */
    gpio_ext[ext_i].setup_output = gpio_ext_pca95xx_setup_output;
    gpio_ext[ext_i].setup_input = gpio_ext_pca95xx_setup_input;
    gpio_ext[ext_i].get = gpio_ext_pca95xx_get;
    gpio_ext[ext_i].set = gpio_ext_pca95xx_set;
    gpio_ext[ext_i].clear = gpio_ext_pca95xx_clear;
    gpio_ext[ext_i].toggle  =gpio_ext_pca95xx_toggle;
    gpio_ext[ext_i].impl_data = &gpio_ext_pca95xx_impl[i];
    /* Set up pca95xx impl struct */
    gpio_ext_pca95xx_impl[i].output_reg = 0x00;
    gpio_ext_pca95xx_impl[i].config_reg = 0x00;
    gpio_ext_pca95xx_impl[i].blocking = blockings[i];
    /* Set up pca95xx peripheral */
    pca95xx_init(&gpio_ext_pca95xx_impl[i].periph, i2c_periphs[i], i2c_addrs[i]);
  }
}


