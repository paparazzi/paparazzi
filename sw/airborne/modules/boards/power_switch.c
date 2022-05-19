/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file "modules/boards/power_switch.c"
 * @author F. van Tienen
 * Simple GPIO power switch module
 */
#include "power_switch.h"
#include "mcu_periph/gpio.h"
#include "generated/airframe.h"
bool power_switch_status = false;

#if !defined(POWER_SWITCH_GPIO)
#error "The power switch module needs POWER_SWITCH_GPIO to be defined"
#endif

/**
 * @brief Intialize the power switch pins
 * This sets the power switch pins to te initial state
 */
void power_switch_init(void) {
  gpio_setup_output(POWER_SWITCH_GPIO);
#ifdef POWER_SWITCH_ENABLE
  power_switch_set(POWER_SWITCH_ENABLE);
#else
  gpio_clear(POWER_SWITCH_GPIO);
#endif
}

/**
 * @brief Set the power switch to enable/disable
 * 
 * @param val True enables the pin, False disables the pin
 */
void power_switch_set(bool val) {
  if(val) {
    gpio_set(POWER_SWITCH_GPIO);
  } else {
    gpio_clear(POWER_SWITCH_GPIO);
  }

  power_switch_status = val;
}
