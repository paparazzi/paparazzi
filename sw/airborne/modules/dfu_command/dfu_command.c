/*
 * Copyright (C) Tom van Dijk
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
 * @file "modules/dfu_command/dfu_command.c"
 * @author Tom van Dijk
 * Read USB serial for dfu command
 */

#include "modules/dfu_command/dfu_command.h"

#include "arch/stm32/mcu_arch.h"

static const char dfu_command_str[] = { 'd', 'f', 'u', '\n' };
static int dfu_command_state = 0;

void dfu_command_periodic(void) {
  while (usb_serial.device.char_available(NULL)) {
    if (usb_serial.device.get_byte(NULL) == dfu_command_str[dfu_command_state]) {
      dfu_command_state++;
      if (dfu_command_state == 4) {
        reset_to_dfu();
      }
    } else {
      dfu_command_state = 0;
    }
  }
}


