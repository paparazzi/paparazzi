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
#include "pprzlink/pprzlink_device.h"
#include "std.h"

#ifndef UART_RX_BUFFER_SIZE
#if defined STM32F4 || defined STM32F7 //the F4 and F7 have enough memory
#define UART_RX_BUFFER_SIZE 256
#else
#define UART_RX_BUFFER_SIZE 128
#endif
#endif

#ifndef UART_TX_BUFFER_SIZE
#if defined STM32F4 || defined STM32F7 //the F4 and F7 have enough memory, and the PX4 bootloader needs more then 128
#define UART_TX_BUFFER_SIZE 256
#else
#define UART_TX_BUFFER_SIZE 128
#endif
#endif

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
  volatile uint8_t tx_running;
  /** UART Register */
  void *reg_addr;
  /** UART Baudrate */
  int baudrate;
  /** User init struct */
  void *init_struct;
  /** UART Dev (linux) */
  char dev[UART_DEV_NAME_SIZE];
  volatile uint16_t ore;    ///< overrun error counter
  volatile uint16_t ne_err; ///< noise error counter
  volatile uint16_t fe_err; ///< framing error counter
  /** Generic device interface */
  struct link_device device;
};


extern void uart_periph_init(struct uart_periph *p);
extern void uart_periph_set_baudrate(struct uart_periph *p, uint32_t baud);
extern void uart_periph_set_bits_stop_parity(struct uart_periph *p, uint8_t bits, uint8_t stop, uint8_t parity);
extern void uart_periph_set_mode(struct uart_periph *p, bool tx_enabled, bool rx_enabled, bool hw_flow_control);
extern void uart_periph_invert_data_logic(struct uart_periph *p, bool invert_rx, bool invert_tx);
extern void uart_put_byte(struct uart_periph *p, long fd, uint8_t data);
extern void uart_put_buffer(struct uart_periph *p, long fd, const uint8_t *data, uint16_t len);
extern int uart_check_free_space(struct uart_periph *p, long *fd, uint16_t len);
extern void uart_send_message(struct uart_periph *p, long fd);
extern uint8_t uart_getch(struct uart_periph *p);

/**
 * Check UART for available chars in receive buffer.
 * @return number of chars in the buffer
 */
extern int uart_char_available(struct uart_periph *p);


extern void uart_arch_init(void);

#if USE_UART0
extern struct uart_periph uart0;
extern void uart0_init(void);
#endif // USE_UART0

#if USE_UART1
extern struct uart_periph uart1;
extern void uart1_init(void);
#endif // USE_UART1

#if USE_UART2
extern struct uart_periph uart2;
extern void uart2_init(void);
#endif // USE_UART2

#if USE_UART3
extern struct uart_periph uart3;
extern void uart3_init(void);
#endif // USE_UART3

#if USE_UART4
extern struct uart_periph uart4;
extern void uart4_init(void);
#endif // USE_UART4

#if USE_UART5
extern struct uart_periph uart5;
extern void uart5_init(void);
#endif // USE_UART5

#if USE_UART6
extern struct uart_periph uart6;
extern void uart6_init(void);
#endif // USE_UART6

#if USE_UART7
extern struct uart_periph uart7;
extern void uart7_init(void);
#endif // USE_UART7

#if USE_UART8
extern struct uart_periph uart8;
extern void uart8_init(void);
#endif // USE_UART8

#endif /* MCU_PERIPH_UART_H */
