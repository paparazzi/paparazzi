/*
 * Paparazzi mcu0 cmx469 modem functions
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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "modem.h"
#include "std.h"

uint8_t modem_nb_ovrn;

uint8_t           tx_head;
volatile uint8_t  tx_tail;
uint8_t           tx_buf[ TX_BUF_SIZE ];

uint8_t    tx_byte;
uint8_t    tx_byte_idx;

uint8_t ck_a, ck_b;

void modem_init( void ) {
  MODEM_OSC_DDR |= _BV(MODEM_OSC);
  OCR0 = 1; /* 4MhZ */
  TCCR0 = _BV(WGM01) | _BV(COM00) | _BV(CS00);

  /* setup TX_EN and TX_DATA pin as output */
  MODEM_TX_DDR |= _BV(MODEM_TX_EN) | _BV(MODEM_TX_DATA);
  /* data idles hight */
  sbi(MODEM_TX_PORT, MODEM_TX_DATA);
  /* enable transmitter */
  cbi(MODEM_TX_PORT, MODEM_TX_EN);
  /* set interrupt on failing edge of clock */
  MODEM_CLK_INT_REG |=  MODEM_CLK_INT_CFG;
}

SIGNAL( MODEM_CLK_INT_SIG ) {
  /*  start bit         */
  if (tx_byte_idx == 0)
    cbi(MODEM_TX_PORT, MODEM_TX_DATA);
  /* 8 data bits        */ 
  else if (tx_byte_idx < 9) {
    if (tx_byte & 0x01)
      sbi(MODEM_TX_PORT, MODEM_TX_DATA);
    else
      cbi(MODEM_TX_PORT, MODEM_TX_DATA);
    tx_byte >>= 1;
  }
  /* stop_bit           */
  else {
    sbi(MODEM_TX_PORT, MODEM_TX_DATA); 
  }
  tx_byte_idx++;
  /* next byte          */
  if (tx_byte_idx >= 10) {
    /*  if we have nothing left to transmit */
    if( tx_head == tx_tail ) {
      /* disable clock interrupt            */
      cbi( EIMSK, MODEM_CLK_INT );
    } else {
      /* else load next byte                  */
      MODEM_LOAD_NEXT_BYTE();  
    }
  }
}
