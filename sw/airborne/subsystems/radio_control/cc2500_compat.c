/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This code is based on the betaflight cc2500 and FrskyX implementation.
 * https://github.com/betaflight/betaflight
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

#include "subsystems/radio_control/cc2500_compat.h"

#include "mcu_periph/gpio.h"

#include <stdbool.h>
#include <assert.h>

// main/rx/rx.h:
rssiSource_e rssiSource;

void bf_setRssi(uint16_t rssiValue, rssiSource_e source) {
  (void) rssiValue;
  (void) source;
}


// main/drivers/io.h:
IO_t bf_IOGetByTag(ioTag_t io) {
  return (IO_t)io;
}

void bf_IOInit(IO_t io, uint8_t owner, uint8_t index) {
  (void) io;
  (void) owner;
  (void) index;
}

void bf_IOConfigGPIO(IO_t io, uint8_t cfg) {
  assert(cfg == IOCFG_IN_FLOATING);
  gpio_setup_input(io->port, io->pin);
}

bool bf_IORead(IO_t gpio) {
  return gpio_get(gpio->port, gpio->pin);
}
