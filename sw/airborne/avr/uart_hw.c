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

/** \file uart_hw.c
 *  \brief avr uart low level functions
 *
 */
#include "uart.h"
#include "sys_time.h"

#define B2400  2400UL
#define B9600  9600UL
#define B38400 38400UL

#if defined  (__AVR_ATmega8__)

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

void uart0_init_tx( void ) {
  UBRRH = 0; 
  UBRRL = F_CPU/(16*UART0_BAUD)-1;

  /* single speed */ 
  UCSRA = 0; 
  /* Enable transmitter */ 
  UCSRB = _BV(TXEN); 
  /* Set frame format: 8data, 1stop bit */ 
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0); 
}

void uart0_init_rx( void ) {
  /* Enable receiver               */ 
  UCSRB |= _BV(RXEN); 
  /* Enable uart receive interrupt */
  sbi( UCSRB, RXCIE ); 
}

bool_t uart0_check_free_space( uint8_t len) {
  int8_t space;
  if ((space = (tx_tail - tx_head)) <= 0)
    space += TX_BUF_SIZE;
  
  return (uint8_t)(space - 1) >= len;
}

void uart0_transmit( unsigned char data ) {
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


SIGNAL(SIG_UART_TRANS) {
  if (tx_head == tx_tail) {
    /* Nothing more to send */
    cbi(UCSRB, TXCIE); /* disable interrupt */
  } else {
    UDR = tx_buf[tx_tail];
    tx_tail++; /* warning tx_buf_len is 256 */
  }
}

#endif /*  (__AVR_ATmega8__) */


#if defined (__AVR_ATmega128__)

#define TX_BUF_SIZE      256

#ifdef USE_UART0
static uint8_t           tx_head0; /* next free in buf */
static volatile uint8_t  tx_tail0; /* next char to send */
uint8_t           tx_buf0[ TX_BUF_SIZE ];


void uart0_init_tx( void ) {
  UBRR0H = 0;
  UBRR0L = F_CPU/(16*UART0_BAUD)-1;

  /* single speed */ 
  UCSR0A = 0; 
  /* Enable transmitter */ 
  UCSR0B = _BV(TXEN);
  /* Set frame format: 8data, 1stop bit */ 
  UCSR0C = _BV(UCSZ1) | _BV(UCSZ0);

  tx_head0 = 0;
  tx_tail0 = 0;
}

void uart0_init_rx( void ) {
  /* Enable receiver */ 
  UCSR0B |= _BV(RXEN);
  
  /* Enable uart receive interrupt */
  sbi(UCSR0B, RXCIE );
}

bool_t uart0_check_free_space( uint8_t len) {
  int8_t space;
  if ((space = (tx_tail0 - tx_head0)) <= 0)
    space += TX_BUF_SIZE;
  
  return (uint16_t)(space - 1) >= len;
}

void uart0_transmit( unsigned char data ) {
  if (UCSR0B & _BV(TXCIE)) {
    /* we are waiting for the last char to be sent : buffering */
    if (tx_tail0 == tx_head0 + 1) { /* BUF_SIZE = 256 */
      /* Buffer is full (almost, but tx_head = tx_tail means "empty" */
      return;
    }
    tx_buf0[tx_head0] = data;
    tx_head0++; /* BUF_SIZE = 256 */
  } else { /* Channel is free: just send */
    UDR0 = data;
    sbi(UCSR0B, TXCIE);
  }
}

SIGNAL(SIG_UART0_TRANS) {
  if (tx_head0 == tx_tail0) {
    /* Nothing more to send */
    cbi(UCSR0B, TXCIE); /* disable interrupt */
  } else {
    UDR0 = tx_buf0[tx_tail0];
    tx_tail0++; /* warning tx_buf_len is 256 */
  }
}

uint8_t uart0_char;
bool_t uart0_char_available;

SIGNAL( SIG_UART0_RECV ) {
  uart0_char = UDR0;
  uart0_char_available = TRUE;
}

#endif /** USE_UART0 */

#ifdef USE_UART1

static uint8_t           tx_head1; /* next free in buf */
static volatile uint8_t  tx_tail1; /* next char to send */
static uint8_t           tx_buf1[ TX_BUF_SIZE ];

void uart1_init_tx( void ) {
  /* set baud rate */
  UBRR1H = 0; 
  UBRR1L = F_CPU/(16*UART1_BAUD)-1;

  /* single speed */ 
  UCSR1A = 0; 
  /* Enable transmitter */ 
  UCSR1B = _BV(TXEN);
  /* Set frame format: 8data, 1stop bit */ 
  UCSR1C = _BV(UCSZ1) | _BV(UCSZ0);

  tx_head1 = 0;
  tx_tail1 = 0;
}

void uart1_init_rx( void ) {
  /* Enable receiver */ 
  UCSR1B |= _BV(RXEN);
  /* Enable uart receive interrupt */
  sbi(UCSR1B, RXCIE ); 
}

bool_t uart1_check_free_space( uint8_t len) {
  int8_t space;
  if ((space = (tx_tail1 - tx_head1)) <= 0)
    space += TX_BUF_SIZE;
  
  return (uint16_t)(space - 1) >= len;
}

void uart1_transmit( unsigned char data ) {
  if (UCSR1B & _BV(TXCIE)) {
    /* we are waiting for the last char to be sent : buffering */
    if (tx_tail1 == tx_head1 + 1) { /* BUF_SIZE = 256 */
      /* Buffer is full (almost, but tx_head = tx_tail means "empty" */
      return;
    }
    tx_buf1[tx_head1] = data;
    tx_head1++; /* BUF_SIZE = 256 */
  } else { /* Channel is free: just send */
    UDR1 = data;
    sbi(UCSR1B, TXCIE);
  }
}


SIGNAL(SIG_UART1_TRANS) {
  if (tx_head1 == tx_tail1) {
    /* Nothing more to send */
    cbi(UCSR1B, TXCIE); /* disable interrupt */
  } else {
    UDR1 = tx_buf1[tx_tail1];
    tx_tail1++; /* warning tx_buf_len is 256 */
  }
}

uint8_t uart1_char;
bool_t uart1_char_available;

SIGNAL( SIG_UART1_RECV ) {
  uart1_char = UDR1;
  uart1_char_available = TRUE;
}

#endif /* USE_UART1 */

#endif /* (__AVR_ATmega128__) */


