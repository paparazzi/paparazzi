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

#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/io.h>


#include "std.h"
#include "uart_fbw.h"

#define TX_BUF_SIZE      256
static uint8_t           tx_head; /* next free in buf */
static volatile uint8_t  tx_tail; /* next char to send */
static uint8_t           tx_buf[ TX_BUF_SIZE ];

/*
 * UART Baud rate generation settings:
 *
 * With 16.0 MHz clock,UBRR=25  => 38400 baud
 *
 */
void uart_init_tx( void ) {
  /* Baudrate is 38.4k */
  UBRRH = 0; 
  UBRRL = 25; 
  /* single speed */ 
  UCSRA = 0; 
  /* Enable transmitter */ 
  UCSRB = _BV(TXEN); 
  /* Set frame format: 8data, 1stop bit */ 
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0); 
}

void uart_init_rx() {
  /* Enable receiver               */ 
  UCSRB |= _BV(RXEN); 
  /* Enable uart receive interrupt */
  sbi( UCSRB, RXCIE ); 
}

void uart_transmit( unsigned char data ) {
  if (UCSRB & _BV(TXCIE)) {
    /* we are waiting for the last char to be sent : buffering */
    if (tx_tail == tx_head + 1) { /* BUF_SIZE = 256 */
      /* Buffer is full (almost, but tx_head = tx_tail means "empty" */
      return;
    }
    tx_buf[tx_head] = data;
    tx_head++; /* BUF_SIZE = 256 */
  } else { /* Channel is free: just send */
    UDR = data;
    sbi(UCSRB, TXCIE);
  }
}

void uart_print_hex ( uint8_t c ) {
  const uint8_t hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', 
                            '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  uint8_t high = (c & 0xF0)>>4;
  uint8_t low  = c & 0x0F;
  uart_transmit(hex[high]);
  uart_transmit(hex[low]);
} 

void uart_print_hex16 ( uint16_t c ) {
  uint8_t high = (uint8_t)(c>>8);
  uint8_t low  = (uint8_t)(c);
  uart_print_hex(high);
  uart_print_hex(low);
}

void uart_print_string(const uint8_t* s) {
  uint8_t i = 0;
  while (s[i]) {
    uart_transmit(s[i]);
    i++;
  }
}

SIGNAL(SIG_UART_TRANS) {
  if (tx_head == tx_tail) {
    /* Nothing more to send */
    cbi(UCSRB, TXCIE); /* disable interrupt */
  } else {
    UDR = tx_buf[tx_tail];
    tx_tail++; /* warning tx_buf_len is 256 */
  }
}
