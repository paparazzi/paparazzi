/*
 * Copyright (C) Freek van Tienen
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
 * @file "modules/com/uart_drop.c"
 * @author Freek van Tienen
 * Module for dropping balls using UART
 */

#include "modules/switching/switch_uart.h"
#include "mcu_periph/uart.h"

uint8_t switch_uart_channel = 0;
uint8_t switch_uart_status = 0;

static uint8_t drop_string[] = "<Drop_Paintball_Now";
#define DROP_STRINGLEN 19


void periodic_switch_uart(void)
{
  while (uart_char_available(&SWITCH_UART_PORT)) {
    uint8_t r = uart_getch(&SWITCH_UART_PORT);
    switch_uart_status = r;
  }
}

void drop_ball(uint8_t number)
{
  for (uint8_t i = 0; i < DROP_STRINGLEN; i++) {
    uart_put_byte(&SWITCH_UART_PORT, 0, drop_string[i]);
  }

  // Drop next ball
  uint8_t last = '>';

  // Or Drop a specific ball
  if (number == 1) {
    last = '1';
  } else if (number == 2) {
    last = '2';
  } else if (number == 3) {
    last = '3';
  } else if (number == 4) {
    last = '4';
  }
  uart_put_byte(&SWITCH_UART_PORT, 0, last);

}
