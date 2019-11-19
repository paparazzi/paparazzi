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
#include "mcu_periph/sys_time.h"

#include <stdbool.h>
#include <assert.h>

// main/drivers/time.h:
void bf_delayMicroseconds(timeUs_t us) {
  assert(us <= UINT32_MAX);
  sys_time_usleep((uint32_t)us);
//  float start = get_sys_time_float();
//  while(get_sys_time_float() < start + (us / 1.0e6)) ;
}

void bf_delay(timeMs_t ms) {
  bf_delayMicroseconds((uint64_t)ms * 1000);
}

timeMs_t bf_millis(void) {
  return get_sys_time_msec();
}


// main/rx/rx.h:
static rxRuntimeState_t runtimeState;

rxRuntimeState_t* rxRuntimeState(void) {
  return &runtimeState;
}

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

void bf_IOConfigGPIO(IO_t io, enum ioconfig_t cfg) {
  if (!io) return;
  switch(cfg) {
    case IOCFG_OUT_PP:
      gpio_setup_output(io->port, io->pin);
      break;
    case IOCFG_IN_FLOATING:
      gpio_setup_input(io->port, io->pin);
      break;
    case IOCFG_IPU:
      gpio_setup_input_pullup(io->port, io->pin);
      break;
    default:
      assert("Invalid IO config" == NULL);
      break;
  }
}

bool bf_IORead(IO_t gpio) {
  if (!gpio) return 0;
  return gpio_get(gpio->port, gpio->pin);
}

void bf_IOHi(IO_t io) {
  if (!io) return;
  gpio_set(io->port, io->pin);
}

void bf_IOLo(IO_t io) {
  if (!io) return;
  gpio_clear(io->port, io->pin);
}

void bf_IOToggle(IO_t io) {
  if (!io) return;
  gpio_toggle(io->port, io->pin);
}

