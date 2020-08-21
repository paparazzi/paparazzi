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

/** @file "modules/gpio_ext/gpio_ext_common.h"
 * @author Tom van Dijk <tomvand@users.noreply.github.com>
 * Common external GPIO functions.
 */

#ifndef GPIO_EXT_COMMON_H
#define GPIO_EXT_COMMON_H


#include <stdint.h>


// External GPIO implementation struct
struct gpio_ext_functions {
  void (*setup_output)(uint32_t port, uint32_t gpios);
  void (*setup_input)(uint32_t port, uint32_t gpios);
  uint32_t (*get)(uint32_t port, uint32_t gpios);
  void (*set)(uint32_t port, uint32_t gpios);
  void (*clear)(uint32_t port, uint32_t gpios);
  void (*toggle)(uint32_t port, uint32_t gpios);
};


#endif  // GPIO_EXT_COMMON_H
