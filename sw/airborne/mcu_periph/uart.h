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
#include "mcu_periph/link_device.h"
#include "std.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128
#define UART_DEV_NAME_SIZE 16

/*
 * UART Baud rate defines in arch/x/mcu_periph/uart_arch.h
 */

#define UBITS_7 7
#define UBITS_8 8

#define USTOP_1 1
#define USTOP_2 2

#define UPARITY_NO    0
#define UPARITY_ODD   1
#define UPARITY_EVEN  2

/**
 * UART peripheral
 */
struct uart_periph {
  /** Receive buffer */
  uint8_t rx_buf[UART_RX_BUFFER_SIZE];
  uint16_t rx_insert_idx;
  uint16_t rx_extract_idx;
  /** Transmit buffer */
  uint8_t tx_buf[UART_TX_BUFFER_SIZE];
  uint16_t tx_insert_idx;
  uint16_t tx_extract_idx;
  uint8_t tx_running;
  /** UART Register */
  void* reg_addr;
  /** UART Dev (linux) */
  char dev[UART_DEV_NAME_SIZE];
  volatile uint16_t ore;    ///< overrun error counter
  volatile uint16_t ne_err; ///< noise error counter
  volatile uint16_t fe_err; ///< framing error counter
  /** Generic device interface */
  struct link_device device;
};


extern void uart_periph_init(struct uart_periph* p);
extern void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud);
extern void uart_periph_set_bits_stop_parity(struct uart_periph* p, uint8_t bits, uint8_t stop, uint8_t parity);
extern void uart_periph_set_mode(struct uart_periph* p, bool_t tx_enabled, bool_t rx_enabled, bool_t hw_flow_control);
extern void uart_transmit(struct uart_periph* p, uint8_t data);
extern bool_t uart_check_free_space(struct uart_periph* p, uint8_t len);
extern uint8_t uart_getch(struct uart_periph* p);
extern void uart_event(void);

/**
 * Check UART for available chars in receive buffer.
 * @return number of chars in the buffer
 */
static inline uint16_t uart_char_available(struct uart_periph* p) {
  int16_t available = p->rx_insert_idx - p->rx_extract_idx;
  if (available < 0)
    available += UART_RX_BUFFER_SIZE;
  return (uint16_t)available;
}


#if USE_UART0
extern struct uart_periph uart0;
extern void uart0_init(void);

#define UART0Init() uart_periph_init(&uart0)
#define UART0CheckFreeSpace(_x) uart_check_free_space(&uart0, _x)
#define UART0Transmit(_x) uart_transmit(&uart0, _x)
#define UART0SendMessage() {}
#define UART0ChAvailable() uart_char_available(&uart0)
#define UART0Getch() uart_getch(&uart0)
#define UART0TxRunning uart0.tx_running
#define UART0SetBaudrate(_b) uart_periph_set_baudrate(&uart0, _b)
#define UART0SetBitsStopParity(_b, _s, _p) uart_periph_set_bits_stop_parity(&uart0, _b, _s, _p)

#endif // USE_UART0

#if USE_UART1
extern struct uart_periph uart1;
extern void uart1_init(void);

#define UART1Init() uart_periph_init(&uart1)
#define UART1CheckFreeSpace(_x) uart_check_free_space(&uart1, _x)
#define UART1Transmit(_x) uart_transmit(&uart1, _x)
#define UART1SendMessage() {}
#define UART1ChAvailable() uart_char_available(&uart1)
#define UART1Getch() uart_getch(&uart1)
#define UART1TxRunning uart1.tx_running
#define UART1SetBaudrate(_b) uart_periph_set_baudrate(&uart1, _b)
#define UART1SetBitsStopParity(_b, _s, _p) uart_periph_set_bits_stop_parity(&uart1, _b, _s, _p)

#endif // USE_UART1

#if USE_UART2
extern struct uart_periph uart2;
extern void uart2_init(void);

#define UART2Init() uart_periph_init(&uart2)
#define UART2CheckFreeSpace(_x) uart_check_free_space(&uart2, _x)
#define UART2Transmit(_x) uart_transmit(&uart2, _x)
#define UART2SendMessage() {}
#define UART2ChAvailable() uart_char_available(&uart2)
#define UART2Getch() uart_getch(&uart2)
#define UART2TxRunning uart2.tx_running
#define UART2SetBaudrate(_b) uart_periph_set_baudrate(&uart2, _b)
#define UART2SetBitsStopParity(_b, _s, _p) uart_periph_set_bits_stop_parity(&uart2, _b, _s, _p)

#endif // USE_UART2

#if USE_UART3
extern struct uart_periph uart3;
extern void uart3_init(void);

#define UART3Init() uart_periph_init(&uart3)
#define UART3CheckFreeSpace(_x) uart_check_free_space(&uart3, _x)
#define UART3Transmit(_x) uart_transmit(&uart3, _x)
#define UART3SendMessage() {}
#define UART3ChAvailable() uart_char_available(&uart3)
#define UART3Getch() uart_getch(&uart3)
#define UART3TxRunning uart3.tx_running
#define UART3SetBaudrate(_b) uart_periph_set_baudrate(&uart3, _b)
#define UART3SetBitsStopParity(_b, _s, _p) uart_periph_set_bits_stop_parity(&uart3, _b, _s, _p)

#endif // USE_UART3

#if USE_UART4
extern struct uart_periph uart4;
extern void uart4_init(void);

#define UART4Init() uart_periph_init(&uart4)
#define UART4CheckFreeSpace(_x) uart_check_free_space(&uart4, _x)
#define UART4Transmit(_x) uart_transmit(&uart4, _x)
#define UART4SendMessage() {}
#define UART4ChAvailable() uart_char_available(&uart4)
#define UART4Getch() uart_getch(&uart4)
#define UART4TxRunning uart4.tx_running
#define UART4SetBaudrate(_b) uart_periph_set_baudrate(&uart4, _b)
#define UART4SetBitsStopParity(_b, _s, _p) uart_periph_set_bits_stop_parity(&uart4, _b, _s, _p)

#endif // USE_UART4

#if USE_UART5
extern struct uart_periph uart5;
extern void uart5_init(void);

#define UART5Init() uart_periph_init(&uart5)
#define UART5CheckFreeSpace(_x) uart_check_free_space(&uart5, _x)
#define UART5Transmit(_x) uart_transmit(&uart5, _x)
#define UART5SendMessage() {}
#define UART5ChAvailable() uart_char_available(&uart5)
#define UART5Getch() uart_getch(&uart5)
#define UART5TxRunning uart5.tx_running
#define UART5SetBaudrate(_b) uart_periph_set_baudrate(&uart5, _b)
#define UART5SetBitsStopParity(_b, _s, _p) uart_periph_set_bits_stop_parity(&uart5, _b, _s, _p)

#endif // USE_UART5

#if USE_UART6
extern struct uart_periph uart6;
extern void uart6_init(void);

#define UART6Init() uart_periph_init(&uart6)
#define UART6CheckFreeSpace(_x) uart_check_free_space(&uart6, _x)
#define UART6Transmit(_x) uart_transmit(&uart6, _x)
#define UART6SendMessage() {}
#define UART6ChAvailable() uart_char_available(&uart6)
#define UART6Getch() uart_getch(&uart6)
#define UART6TxRunning uart6.tx_running
#define UART6SetBaudrate(_b) uart_periph_set_baudrate(&uart6, _b)
#define UART6SetBitsStopParity(_b, _s, _p) uart_periph_set_bits_stop_parity(&uart6, _b, _s, _p)

#endif // USE_UART6

#endif /* MCU_PERIPH_UART_H */
