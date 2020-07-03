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
 *
 * The reset_to_dfu() function needs to be implemented for the
 * architecture in use. (sw/airborne/arch/.../mcu_arch.c)
 */

#include "modules/dfu_command/dfu_command.h"

#include "mcu_arch.h"

static const char dfu_command_str[] = "#\nbl\n"; // Note: same command resets betaflight to DFU
static int dfu_command_state = 0;

void dfu_command_event(void) {
  // Search fifo for command string
  for (int i = 0; i < VCOM_check_available(); ++i) {
    if (VCOM_peekchar(i) == dfu_command_str[dfu_command_state]) {
      dfu_command_state++;
      if (dfu_command_str[dfu_command_state] == '\0') { // end of command string
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


