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
#if USE_UART2
#define TEST_UART uart2
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
  static uint8_t i = 0;

  /* start "packet with zero */
  //uart_put_byte(&TEST_UART, 0);
  uart_put_byte(&TEST_UART, i);
  /* print status every x cycles */
  RunOnceEvery(1, printf("%f, transmit: '%d'\n", get_sys_time_float(), i););

  i++;
}

static inline void main_event(void)
{
}
