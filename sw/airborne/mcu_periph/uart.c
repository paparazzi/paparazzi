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

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#if USE_UART0
struct uart_periph uart0;

#if PERIODIC_TELEMETRY
static void send_uart0_err(void) {
  uint16_t ore    = uart0.ore;
  uint16_t ne_err = uart0.ne_err;
  uint16_t fe_err = uart0.fe_err;
  const uint8_t _bus0 = 0;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus0);
}
#endif

#endif

#if USE_UART1
struct uart_periph uart1;

#if PERIODIC_TELEMETRY
static void send_uart1_err(void) {
  uint16_t ore    = uart1.ore;
  uint16_t ne_err = uart1.ne_err;
  uint16_t fe_err = uart1.fe_err;
  const uint8_t _bus1 = 1;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus1);
}
#endif

#endif

#if USE_UART2
struct uart_periph uart2;

#if PERIODIC_TELEMETRY
static void send_uart2_err(void) {
  uint16_t ore    = uart2.ore;
  uint16_t ne_err = uart2.ne_err;
  uint16_t fe_err = uart2.fe_err;
  const uint8_t _bus2 = 2;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus2);
}
#endif

#endif

#if USE_UART3
struct uart_periph uart3;

#if PERIODIC_TELEMETRY
static void send_uart3_err(void) {
  uint16_t ore    = uart3.ore;
  uint16_t ne_err = uart3.ne_err;
  uint16_t fe_err = uart3.fe_err;
  const uint8_t _bus3 = 3;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus3);
}
#endif

#endif

#if USE_UART4
struct uart_periph uart4;

#if PERIODIC_TELEMETRY
static void send_uart4_err(void) {
  uint16_t ore    = uart4.ore;
  uint16_t ne_err = uart4.ne_err;
  uint16_t fe_err = uart4.fe_err;
  const uint8_t _bus4 = 4;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus4);
}
#endif

#endif

#if USE_UART5
struct uart_periph uart5;

#if PERIODIC_TELEMETRY
static void send_uart5_err(void) {
  uint16_t ore    = uart5.ore;
  uint16_t ne_err = uart5.ne_err;
  uint16_t fe_err = uart5.fe_err;
  const uint8_t _bus5 = 5;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus5);
}
#endif

#endif

#if USE_UART6
struct uart_periph uart6;

#if PERIODIC_TELEMETRY
static void send_uart6_err(void) {
  const uint8_t _bus6 = 6;
  uint16_t ore    = uart6.ore;
  uint16_t ne_err = uart6.ne_err;
  uint16_t fe_err = uart6.fe_err;
  DOWNLINK_SEND_UART_ERRORS(DefaultChannel, DefaultDevice,
      &ore, &ne_err, &fe_err, &_bus6);
}
#endif

#endif

#if PERIODIC_TELEMETRY
static void send_uart_err(void) {
  static uint8_t uart_nb_cnt = 0;
  switch (uart_nb_cnt) {
#if USE_UART0
    case 0:
      send_uart0_err(); break;
#endif
#if USE_UART1
    case 1:
      send_uart1_err(); break;
#endif
#if USE_UART2
    case 2:
      send_uart2_err(); break;
#endif
#if USE_UART3
    case 3:
      send_uart3_err(); break;
#endif
#if USE_UART4
    case 4:
      send_uart4_err(); break;
#endif
#if USE_UART5
    case 5:
      send_uart5_err(); break;
#endif
#if USE_UART6
    case 6:
      send_uart6_err(); break;
#endif
    default: break;
  }
  uart_nb_cnt++;
  if (uart_nb_cnt == 7)
    uart_nb_cnt = 0;
}
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

#if PERIODIC_TELEMETRY
  // the first to register do it for the others
  register_periodic_telemetry(DefaultPeriodic, "UART_ERRORS", send_uart_err);
#endif
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
