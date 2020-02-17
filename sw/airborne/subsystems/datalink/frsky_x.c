/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING. If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "frsky_x.h"

#include "subsystems/radio_control/cc2500_frsky/cc2500_smartport.h"

#include <string.h>

static uint32_t counter = 0;

struct frsky_x_serial_periph frsky_x_serial;

static bool smartPortDownlink_cb(uint32_t *data) {
  ++counter;
  *data = counter;
  return true;
}

void datalink_frsky_x_init(void) {
  frsky_x_serial.device.periph = (void *)(&frsky_x_serial);
  frsky_x_serial.device.check_free_space = NULL;
  frsky_x_serial.device.put_byte = NULL;
  frsky_x_serial.device.put_buffer = NULL;
  frsky_x_serial.device.send_message = NULL;
  frsky_x_serial.device.char_available = NULL;
  frsky_x_serial.device.get_byte = NULL;

  smartPortDownlink = smartPortDownlink_cb;
}
