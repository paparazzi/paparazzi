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

/** \file mcu_periph/uart.h
 *  \brief arch independent UART (Universal Asynchronous Receiver/Transmitter) API
 *
 */

#ifndef MCU_PERIPH_UART_H
#define MCU_PERIPH_UART_H

#include "mcu_periph/uart_arch.h"
#include "std.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128
#define UART_DEV_NAME_SIZE 16

/*
 * UART Baud rates
 * defines because the stupid c preprocessor can't handle enums
*/
#define B1200    1200
#define B2400    2400
#define B4800    4800
#define B9600    9600
#define B19200   19200
#define B38400   38400
#define B57600   57600
#define B115200  115200
#define B230400  230400

/**
 * UART peripheral
 */
struct uart_periph {
  /* Receive buffer */
  uint8_t rx_buf[UART_RX_BUFFER_SIZE];
  uint16_t rx_insert_idx;
  uint16_t rx_extract_idx;
  /* Transmit buffer */
  uint8_t tx_buf[UART_TX_BUFFER_SIZE];
  uint16_t tx_insert_idx;
  uint16_t tx_extract_idx;
  uint8_t tx_running;
  /* UART Register */
  void* reg_addr;
  /* UART Dev (linux) */
  char dev[UART_DEV_NAME_SIZE];
};

extern void uart_periph_init(struct uart_periph* p);
extern void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud, bool_t hw_flow_control);
extern void uart_transmit(struct uart_periph* p, uint8_t data);
extern bool_t uart_check_free_space(struct uart_periph* p, uint8_t len);
extern uint8_t uart_getch(struct uart_periph* p);

static inline bool_t uart_char_available(struct uart_periph* p) {
  return (p->rx_insert_idx != p->rx_extract_idx);
}


#ifdef USE_UART0
extern struct uart_periph uart0;
extern void uart0_init(void);

#define UART0Init() uart_periph_init(&uart0)
#define UART0CheckFreeSpace(_x) uart_check_free_space(&uart0, _x)
#define UART0Transmit(_x) uart_transmit(&uart0, _x)
#define UART0SendMessage() {}
#define UART0ChAvailable() uart_char_available(&uart0)
#define UART0Getch() uart_getch(&uart0)
#define UART0TxRunning uart0.tx_running
#define UART0SetBaudrate(_b) uart_periph_set_baudrate(&uart0, _b, FALSE)

#endif // USE_UART0

#ifdef USE_UART1
extern struct uart_periph uart1;
extern void uart1_init(void);

#define UART1Init() uart_periph_init(&uart1)
#define UART1CheckFreeSpace(_x) uart_check_free_space(&uart1, _x)
#define UART1Transmit(_x) uart_transmit(&uart1, _x)
#define UART1SendMessage() {}
#define UART1ChAvailable() uart_char_available(&uart1)
#define UART1Getch() uart_getch(&uart1)
#define UART1TxRunning uart1.tx_running
#if UART1_HW_FLOW_CONTROL
#define UART1SetBaudrate(_b) uart_periph_set_baudrate(&uart1, _b, TRUE)
#else
#define UART1SetBaudrate(_b) uart_periph_set_baudrate(&uart1, _b, FALSE)
#endif

#endif // USE_UART1

#ifdef USE_UART2
extern struct uart_periph uart2;
extern void uart2_init(void);

#define UART2Init() uart_periph_init(&uart2)
#define UART2CheckFreeSpace(_x) uart_check_free_space(&uart2, _x)
#define UART2Transmit(_x) uart_transmit(&uart2, _x)
#define UART2SendMessage() {}
#define UART2ChAvailable() uart_char_available(&uart2)
#define UART2Getch() uart_getch(&uart2)
#define UART2TxRunning uart2.tx_running
#define UART2SetBaudrate(_b) uart_periph_set_baudrate(&uart2, _b, FALSE)

#endif // USE_UART2

#ifdef USE_UART3
extern struct uart_periph uart3;
extern void uart3_init(void);

#define UART3Init() uart_periph_init(&uart3)
#define UART3CheckFreeSpace(_x) uart_check_free_space(&uart3, _x)
#define UART3Transmit(_x) uart_transmit(&uart3, _x)
#define UART3SendMessage() {}
#define UART3ChAvailable() uart_char_available(&uart3)
#define UART3Getch() uart_getch(&uart3)
#define UART3TxRunning uart3.tx_running
#define UART3SetBaudrate(_b) uart_periph_set_baudrate(&uart3, _b, FALSE)

#endif // USE_UART3

#ifdef USE_UART5
extern struct uart_periph uart5;
extern void uart5_init(void);

#define UART5Init() uart_periph_init(&uart5)
#define UART5CheckFreeSpace(_x) uart_check_free_space(&uart5, _x)
#define UART5Transmit(_x) uart_transmit(&uart5, _x)
#define UART5SendMessage() {}
#define UART5ChAvailable() uart_char_available(&uart5)
#define UART5Getch() uart_getch(&uart5)
#define UART5TxRunning uart5.tx_running
#define UART5SetBaudrate(_b) uart_periph_set_baudrate(&uart5, _b, FALSE)

#endif // USE_UART5

#endif /* MCU_PERIPH_UART_H */
