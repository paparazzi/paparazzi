/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "std.h"

#include <stdio.h>
#include <string.h>

#ifndef TEST_UART
#if USE_UART1
#define TEST_UART uart1
#endif
#if USE_UART4
#define TEST_UART uart4
#endif
#endif
PRINT_CONFIG_VAR(TEST_UART)

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

int main(void)
{
  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  }

  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
}

static inline void main_periodic(void)
{
  const char *foo = "FooBarBaz";
  static int i = 0;

  if (i == strlen(foo)) {
    i = 0;
  }

  uart_put_byte(&TEST_UART, foo[i]);
  printf("%f, transmit: '%c'\n", get_sys_time_float(), foo[i]);

  if (uart_char_available(&TEST_UART)) {
    char c =  uart_getch(&TEST_UART);
    printf("%f, received: '%c'\n", get_sys_time_float(), c);
  }

  i++;
}

static inline void main_event(void)
{
}
