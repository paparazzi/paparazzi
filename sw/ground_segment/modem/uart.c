#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "uart.h"


uint8_t uart_nb_ovrrun = 0;

#define TX_BUF_SIZE     100

static volatile uint8_t  tx_head = TX_BUF_SIZE - 1;
static volatile uint8_t  tx_tail = TX_BUF_SIZE - 1;
static uint8_t           tx_buf[ TX_BUF_SIZE ];


/*
 * UART Baud rate generation settings:
 *
 * With 16.0 MHz clock,UBRR=25  => 38400 baud
 * With 8.0 Mhz clock, UBRR=12  => 38400 baud
 *
 * With 4.0 MHz UBRR=12 + ub2X=1 -> 38400 baud 
 */

void uart_init( void ) {
  /* Baudrate is 38.4k */
  UBRRH = 0;
  UBRRL = 12; 
  /* double speed */ 
  UCSRA = _BV(U2X); 
  /* Enable transmitter */ 
  UCSRB = _BV(TXEN); 
  /* Set frame format: 8data, 1stop bit */ 
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0); 
}


static inline void load_next_byte( void ) {
  uint8_t tmp_tail;
  /* load a new byte */
  tmp_tail = tx_tail + 1;
  if( tmp_tail >= TX_BUF_SIZE )
    tmp_tail = 0;
  tx_tail = tmp_tail;
  UDR = tx_buf[tx_tail];
}

void uart_putc( unsigned char c ) {
  uint8_t tmp_head;

  tmp_head = tx_head + 1;
  if( tmp_head >= TX_BUF_SIZE )
    tmp_head = 0;
  /* if buffer is full do nothing */
  if( tmp_head == tx_tail ) {
    uart_nb_ovrrun++;
    return;
  }

  /* copy data to buffer       */
  tx_buf[ tmp_head ] = c;
  /* update head               */
  tx_head = tmp_head;

  /* if we were not allready transmitting */
  if (bit_is_clear(UCSRB, TXCIE)) {
    /* load a byte */
    load_next_byte();
    /* enable interrupt */
    sbi(UCSRB, TXCIE);
  }
}

SIGNAL( SIG_UART_TRANS ) { 
  /*  if we have nothing left to transmit */
  if( tx_head == tx_tail )
    /* disable data register empty interrupt */
    cbi(UCSRB, TXCIE);
  else
    load_next_byte();
}
