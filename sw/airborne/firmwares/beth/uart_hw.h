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

#ifndef UART_HW_H
#define UART_HW_H

#include "std.h"
//coment to avoid redefinition
/*#define B9600     9600
#define B38400   38400
 #define B57600   57600
#define B115200 115200
*/

//junk for gps_configure_uart in gps_ubx.c to compile
#define UART_8N1 1
#define UART_FIFO_8 1

#define UART1_irq_handler usart1_irq_handler
#define UART2_irq_handler usart2_irq_handler
#define UART3_irq_handler usart3_irq_handler
#define UART5_irq_handler usart5_irq_handler

#if USE_UART0
extern void uart0_handler(void);
#endif


#if USE_UART0
#define UART0_RX_BUFFER_SIZE 128
#define UART0_TX_BUFFER_SIZE 128

extern volatile uint16_t uart0_rx_insert_idx, uart0_rx_extract_idx;
extern uint8_t  uart0_rx_buffer[UART0_RX_BUFFER_SIZE];

extern volatile uint16_t uart0_tx_insert_idx, uart0_tx_extract_idx;
extern volatile bool_t   uart0_tx_running;
extern uint8_t  uart0_tx_buffer[UART0_TX_BUFFER_SIZE];

#define UART0ChAvailable() (uart0_rx_insert_idx != uart0_rx_extract_idx)
#define UART0Getch() ({             \
    uint8_t ret = uart0_rx_buffer[uart0_rx_extract_idx];    \
    uart0_rx_extract_idx = (uart0_rx_extract_idx + 1)%UART0_RX_BUFFER_SIZE; \
    ret;                \
  })

#endif /* USE_UART0 */

#if USE_UART1
extern void uart1_handler(void);
#endif


#if USE_UART1
#define UART1_RX_BUFFER_SIZE 128
#define UART1_TX_BUFFER_SIZE 128

extern volatile uint16_t uart1_rx_insert_idx, uart1_rx_extract_idx;
extern uint8_t  uart1_rx_buffer[UART1_RX_BUFFER_SIZE];

extern volatile uint16_t uart1_tx_insert_idx, uart1_tx_extract_idx;
extern volatile bool_t   uart1_tx_running;
extern uint8_t  uart1_tx_buffer[UART1_TX_BUFFER_SIZE];

#define UART1ChAvailable() (uart1_rx_insert_idx != uart1_rx_extract_idx)
#define UART1Getch() ({             \
    uint8_t ret = uart1_rx_buffer[uart1_rx_extract_idx];    \
    uart1_rx_extract_idx = (uart1_rx_extract_idx + 1)%UART1_RX_BUFFER_SIZE; \
    ret;                \
  })

#endif /* USE_UART1 */


void uart_init(void);
void uart0_init_param(uint16_t baud, uint8_t mode, uint8_t fmode);

#endif /* UART_HW_H */
