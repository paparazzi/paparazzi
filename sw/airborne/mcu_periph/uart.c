/*
 * Copyright (C) 2010 The Paparazzi Team
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
 *
 */

#include "mcu_periph/uart.h"

#ifdef USE_UART0
struct uart_periph uart0;
#endif

#ifdef USE_UART1
struct uart_periph uart1;
#endif

#ifdef USE_UART2
struct uart_periph uart2;
#endif

#ifdef USE_UART3
struct uart_periph uart3;
#endif

#ifdef USE_UART4
struct uart_periph uart4;
#endif

#ifdef USE_UART5
struct uart_periph uart5;
#endif

#ifdef USE_UART6
struct uart_periph uart6;
#endif

void uart_periph_init(struct uart_periph* p) {
  p->rx_insert_idx = 0;
  p->rx_extract_idx = 0;
  p->tx_insert_idx = 0;
  p->tx_extract_idx = 0;
  p->tx_running = FALSE;
  p->ore = 0;
  p->ne_err = 0;
  p->fe_err = 0;
}

bool_t uart_check_free_space(struct uart_periph* p, uint8_t len) {
  int16_t space = p->tx_extract_idx - p->tx_insert_idx;
  if (space <= 0)
    space += UART_TX_BUFFER_SIZE;
  return (uint16_t)(space - 1) >= len;
}

uint8_t uart_getch(struct uart_periph* p) {
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  return ret;
}
