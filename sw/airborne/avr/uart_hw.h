/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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

/** \file uart_hw.h
 *  \brief avr uart low level headers
 *
 */

#ifndef UART_HW_H
#define UART_HW_H

#include <avr/io.h>
#if (__GNUC__ == 3)
#include <avr/signal.h>
#endif
#include <avr/interrupt.h>
#include "std.h"


/************************************************************************/
#if defined  (__AVR_ATmega8__)

#define ReceiveUart(cb) \
  SIGNAL( SIG_UART_RECV ) { \
    uint8_t c = UDR; \
    cb(c); \
}

#endif /* (__AVR_ATmega8__) */


/************************************************************************/
#if defined  (__AVR_ATmega128__)

extern uint8_t           tx_buf0[256]; /** For debugging purpose */

extern void uart0_init_tx(void);
extern void uart0_init_rx(void);
extern void uart1_init(void);

extern void uart0_transmit(const uint8_t);
extern void uart1_transmit(const uint8_t);

#define UART0_RX_BUFFER_SIZE 32        // UART0 receive buffer size
#define UART1_RX_BUFFER_SIZE 32        // UART1 receive buffer size
#define UART0_RX_BUFFER_SIZE_MASK 0x1f
#define UART1_RX_BUFFER_SIZE_MASK 0x1f

#ifdef UART0_RX_BUFFER_SIZE_MASK
#define Uart0RxBufferNext(_x) ((_x+1)&UART0_RX_BUFFER_SIZE_MASK)
#else
#define Uart0RxBufferNext(_x) ((_x+1)%UART0_RX_BUFFER_SIZE)
#endif

#ifdef UART1_RX_BUFFER_SIZE_MASK
#define Uart1RxBufferNext(_x) ((_x+1)&UART1_RX_BUFFER_SIZE_MASK)
#else
#define Uart1RxBufferNext(_x) ((_x+1)%UART1_RX_BUFFER_SIZE)
#endif


extern uint16_t uart0_rx_insert_idx, uart0_rx_extract_idx;
extern uint8_t uart0_rx_buffer[UART0_RX_BUFFER_SIZE];

#define Uart0ChAvailable() (uart0_rx_insert_idx != uart0_rx_extract_idx)

#define Uart0Getch() ({\
   uint8_t ret = uart0_rx_buffer[uart0_rx_extract_idx]; \
   uart0_rx_extract_idx = Uart0RxBufferNext(uart0_rx_extract_idx);        \
   ret;                                                 \
})


extern uint16_t uart1_rx_insert_idx, uart1_rx_extract_idx;
extern uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];

#define Uart1ChAvailable() (uart1_rx_insert_idx != uart1_rx_extract_idx)

#define Uart1Getch() ({\
   uint8_t ret = uart1_rx_buffer[uart1_rx_extract_idx]; \
   uart1_rx_extract_idx = Uart1RxBufferNext(uart1_rx_extract_idx);        \
   ret;                                                 \
})

#endif /* (__AVR_ATmega128__) */

#endif /* UART_HW_H */
