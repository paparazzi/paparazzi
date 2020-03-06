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

void dfu_command_event(void) {
  // Search fifo for 'dfu\n' command string
  for (int i = 0; i < VCOM_check_available(); ++i) {
    if (VCOM_peekchar(i) == dfu_command_str[dfu_command_state]) {
      dfu_command_state++;
      if (dfu_command_state == 4) {
        reset_to_dfu();
      }
    } else {
      dfu_command_state = 0;
    }
  }

  // Prevent fifo blocking if bytes are not consumed by other process
  if (!VCOM_check_free_space(1)) {
    while (VCOM_check_available()) {
      VCOM_getchar();
    }
  }
}


