/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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
#include "led.h"

static inline void main_init(void);
static inline void main_periodic(void);

int main(void)
{

  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
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
  char ch;

#if USE_UART1
  uart_put_byte(&uart1, 'a');
#endif
#if USE_UART2
  uart_put_byte(&uart2, 'b');
#endif
#if USE_UART3
  uart_put_byte(&uart3, 'c');
#endif
#if USE_UART4
  uart_put_byte(&uart4, 'd');
#endif
#if USE_UART5
  uart_put_byte(&uart5, 'e');
#endif

  LED_OFF(1);
  LED_OFF(2);

#if USE_UART1
  if (uart_char_available(&uart1)) {
    ch = uart_getch(&uart1);
    if (ch == 'a') {
      LED_ON(1);
    } else {
      LED_ON(2);
    }
  }
#endif

#if USE_UART2
  if (uart_char_available(&uart2)) {
    ch =  uart_getch(&uart2);
    if (ch == 'b') {
      LED_ON(1);
    } else {
      LED_ON(2);
    }
  }
#endif

#if USE_UART3
  if (uart_char_available(&uart3)) {
    ch =  uart_getch(&uart3);
    if (ch == 'c') {
      LED_ON(1);
    } else {
      LED_ON(2);
    }
  }
#endif

#if USE_UART4
  if (uart_char_available(&uart4)) {
    ch =  uart_getch(&uart4);
    if (ch == 'd') {
      LED_ON(1);
    } else {
      LED_ON(2);
    }
  }
#endif

#if USE_UART5
  if (uart_char_available(&uart5)) {
    ch =  uart_getch(&uart5);
    if (ch == 'e') {
      LED_ON(1);
    } else {
      LED_ON(2);
    }
  }
#endif
}
