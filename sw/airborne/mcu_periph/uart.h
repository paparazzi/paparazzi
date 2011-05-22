/*
 * Paparazzi $Id$
 *
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
 *  \brief arch independant UART (Universal Asynchronous Receiver/Transmitter) API
 *
 */

#ifndef MCU_PERIPH_UART_H
#define MCU_PERIPH_UART_H

#include "mcu_periph/uart_arch.h"
#include "std.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128
#define UART_DEV_NAME_SIZE 16

/**
 * UART peripheral
 */
struct uart_periph {
  /* Receive buffer */
  uint8_t rx_buf[UART_RX_BUFFER_SIZE];
  uint16_t rx_insert_idx;
  uint16_t rx_extract_idx;
  /* Transmit buffer */
  uint8_t tx_buf[UART_RX_BUFFER_SIZE];
  uint16_t tx_insert_idx;
  uint16_t tx_extract_idx;
  uint8_t tx_running;
  /* UART Register */
  void* reg_addr;
  /* UART Dev (linux) */
  char dev[UART_DEV_NAME_SIZE];
};

extern void uart_periph_init(struct uart_periph* p);
extern void uart_periph_set_baudrate(struct uart_periph* p, uint32_t baud);
//extern void uart_periph_init_param(struct uart_periph* p, uint32_t baud, uint8_t mode, uint8_t fmode, char * dev);
extern void uart_transmit(struct uart_periph* p, uint8_t data);
extern bool_t uart_check_free_space(struct uart_periph* p, uint8_t len);

#define UartChAvailable(_p) (_p.rx_insert_idx != _p.rx_extract_idx)

#define UartGetch(_p) ({                                            \
   uint8_t ret = _p.rx_buf[_p.rx_extract_idx];                   \
   _p.rx_extract_idx = (_p.rx_extract_idx + 1)%UART_RX_BUFFER_SIZE; \
   ret;                                                             \
})


#ifdef USE_UART0
extern struct uart_periph uart0;
extern void uart0_init(void);

#define Uart0Init() uart_periph_init(&uart0)
#define Uart0CheckFreeSpace(_x) uart_check_free_space(&uart0, _x)
#define Uart0Transmit(_x) uart_transmit(&uart0, _x)
#define Uart0SendMessage() {}
#define Uart0ChAvailable() UartChAvailable(uart0)
#define Uart0Getch() UartGetch(uart0)
#define Uart0TxRunning uart0.tx_running
#define Uart0SetBaudrate(_b) uart_periph_set_baudrate(&uart0, _b)
//#define Uart0InitParam(_b, _m, _fm) uart_periph_init_param(&uart0, _b, _m, _fm, "")

#define UART0Init           Uart0Init
#define UART0CheckFreeSpace Uart0CheckFreeSpace
#define UART0Transmit       Uart0Transmit
#define UART0SendMessage    Uart0SendMessage
#define UART0ChAvailable    Uart0ChAvailable
#define UART0Getch          Uart0Getch
#define UART0TxRunning      Uart0TxRunning
#define UART0InitParam      Uart0InitParam

#endif // USE_UART0

#ifdef USE_UART1
extern struct uart_periph uart1;
extern void uart1_init(void);

#define Uart1Init() uart_periph_init(&uart1)
#define Uart1CheckFreeSpace(_x) uart_check_free_space(&uart1, _x)
#define Uart1Transmit(_x) uart_transmit(&uart1, _x)
#define Uart1SendMessage() {}
#define Uart1ChAvailable() UartChAvailable(uart1)
#define Uart1Getch() UartGetch(uart1)
#define Uart1TxRunning uart1.tx_running
#define Uart1SetBaudrate(_b) uart_periph_set_baudrate(&uart1, _b)
//#define Uart1InitParam(_b, _m, _fm) uart_periph_init_param(&uart1, _b, _m, _fm, "")

#define UART1Init           Uart1Init
#define UART1CheckFreeSpace Uart1CheckFreeSpace
#define UART1Transmit       Uart1Transmit
#define UART1SendMessage    Uart1SendMessage
#define UART1ChAvailable    Uart1ChAvailable
#define UART1Getch          Uart1Getch
#define UART1TxRunning      Uart1TxRunning
#define UART1InitParam      Uart1InitParam

#endif // USE_UART1

#ifdef USE_UART2
extern struct uart_periph uart2;
extern void uart2_init(void);

#define Uart2Init() uart_periph_init(&uart2)
#define Uart2CheckFreeSpace(_x) uart_check_free_space(&uart2, _x)
#define Uart2Transmit(_x) uart_transmit(&uart2, _x)
#define Uart2SendMessage() {}
#define Uart2ChAvailable() UartChAvailable(uart2)
#define Uart2Getch() UartGetch(uart2)
#define Uart2TxRunning uart2.tx_running
#define Uart2SetBaudrate(_b) uart_periph_set_baudrate(&uart2, _b)
//#define Uart2InitParam(_b, _m, _fm) uart_periph_init_param(&uart2, _b, _m, _fm, "")

#define UART2Init           Uart2Init
#define UART2CheckFreeSpace Uart2CheckFreeSpace
#define UART2Transmit       Uart2Transmit
#define UART2SendMessage    Uart2SendMessage
#define UART2ChAvailable    Uart2ChAvailable
#define UART2Getch          Uart2Getch
#define UART2TxRunning      Uart2TxRunning
#define UART2InitParam      Uart2InitParam

#endif // USE_UART2

#ifdef USE_UART3
extern struct uart_periph uart3;
extern void uart3_init(void);

#define Uart3Init() uart_periph_init(&uart3)
#define Uart3CheckFreeSpace(_x) uart_check_free_space(&uart3, _x)
#define Uart3Transmit(_x) uart_transmit(&uart3, _x)
#define Uart3SendMessage() {}
#define Uart3ChAvailable() UartChAvailable(uart3)
#define Uart3Getch() UartGetch(uart3)
#define Uart3TxRunning uart3.tx_running
#define Uart3SetBaudrate(_b) uart_periph_set_baudrate(&uart3, _b)
//#define Uart3InitParam(_b, _m, _fm) uart_periph_init_param(&uart3, _b, _m, _fm, "")

#define UART3Init           Uart3Init
#define UART3CheckFreeSpace Uart3CheckFreeSpace
#define UART3Transmit       Uart3Transmit
#define UART3SendMessage    Uart3SendMessage
#define UART3ChAvailable    Uart3ChAvailable
#define UART3Getch          Uart3Getch
#define UART3TxRunning      Uart3TxRunning
#define UART3InitParam      Uart3InitParam

#endif // USE_UART3

#ifdef USE_UART5
extern struct uart_periph uart5;
extern void uart5_init(void);

#define Uart5Init() uart_periph_init(&uart5)
#define Uart5CheckFreeSpace(_x) uart_check_free_space(&uart5, _x)
#define Uart5Transmit(_x) uart_transmit(&uart5, _x)
#define Uart5SendMessage() {}
#define Uart5ChAvailable() UartChAvailable(uart5)
#define Uart5Getch() UartGetch(uart5)
#define Uart5TxRunning uart5.tx_running
#define Uart5SetBaudrate(_b) uart_periph_set_baudrate(&uart5, _b)
//#define Uart5InitParam(_b, _m, _fm) uart_periph_init_param(&uart5, _b, _m, _fm, "")

#define UART5Init           Uart5Init
#define UART5CheckFreeSpace Uart5CheckFreeSpace
#define UART5Transmit       Uart5Transmit
#define UART5SendMessage    Uart5SendMessage
#define UART5ChAvailable    Uart5ChAvailable
#define UART5Getch          Uart5Getch
#define UART5TxRunning      Uart5TxRunning
#define UART5InitParam      Uart5InitParam

#endif // USE_UART5

#endif /* MCU_PERIPH_UART_H */
