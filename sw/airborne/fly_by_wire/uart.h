/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

#ifndef _UART_H_
#define _UART_H_

#include <inttypes.h>

void uart_init_tx( void );
void uart_init_rx( void );
void uart_transmit( unsigned char data );

void uart_print_hex ( uint8_t c );
void uart_print_hex16 ( uint16_t c );
void uart_print_string(const uint8_t* s);
void uart_print_float( const float * f);

#define TX_BUF_SIZE     256
extern uint8_t           tx_head;
extern volatile uint8_t  tx_tail;
extern uint8_t           tx_buf[ TX_BUF_SIZE ];

#define UART_CHECK_FREE_SPACE(_space) (tx_head>=tx_tail? _space < (TX_BUF_SIZE - (tx_head - tx_tail)) : _space < (tx_tail - tx_head))

#define UART_PUT_1_BYTE(_byte) { \
  tx_buf[tx_head] = _byte; \
  tx_head++; \
  if (tx_head >= TX_BUF_SIZE) tx_head = 0; \
}

#define UART_PUT_1_BYTE_BY_ADDR(_byte) { \
    UART_PUT_1_BYTE(*(_byte));		 \
}

#define UART_CHECK_RUNNING() {}

#endif
