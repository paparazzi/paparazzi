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

extern uint8_t uart1_char, uart0_char;
extern bool_t uart1_char_available, uart0_char_available;

#define Uart0ChAvailable() (uart0_char_available)
#define Uart1ChAvailable() (uart1_char_available)

static inline uint8_t Uart0Getch( void ) {
  uart0_char_available = FALSE;
  return uart0_char;
}

static inline uint8_t Uart1Getch( void ) {
  uart1_char_available = FALSE;
  return uart1_char;
}

#endif /* (__AVR_ATmega128__) */

#endif /* UART_HW_H */
