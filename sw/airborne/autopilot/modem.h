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

#ifndef MODEM_H
#define MODEM_H

#include "airframe.h"

void modem_init( void );

extern uint8_t modem_nb_ovrn;

#define TX_BUF_SIZE     255
extern uint8_t           tx_head;
extern volatile uint8_t  tx_tail;
extern uint8_t           tx_buf[ TX_BUF_SIZE ];

extern uint8_t    tx_byte;
extern uint8_t    tx_byte_idx;

extern uint8_t ck_a, ck_b;

#define ModemStartMessage(id) \
  { MODEM_PUT_1_BYTE(STX); MODEM_PUT_1_BYTE(id); ck_a = id; ck_b = id; }

#define ModemEndMessage() \
  { MODEM_PUT_1_BYTE(ck_a); MODEM_PUT_1_BYTE(ck_b); MODEM_CHECK_RUNNING(); }


#define MODEM_TX_PORT   PORTD
#define MODEM_TX_DDR	DDRD
#define MODEM_TX_EN     7
#define MODEM_TX_DATA   6

#ifdef CTL_BRD_V1_1
#define MODEM_CLK_DDR   DDRD
#define MODEM_CLK_PORT  PORTD
#define MODEM_CLK       0
#define MODEM_CLK_INT   INT0
#define MODEM_CLK_INT_REG EICRA
#define MODEM_CLK_INT_CFG _BV(ISC01)
#define MODEM_CLK_INT_SIG SIG_INTERRUPT0
#endif /* CTL_BRD_V1_1 */

#ifdef CTL_BRD_V1_2 
#define MODEM_CLK_DDR   DDRD
#define MODEM_CLK_PORT  PORTD
#define MODEM_CLK       0
#define MODEM_CLK_INT   INT0
#define MODEM_CLK_INT_REG EICRA
#define MODEM_CLK_INT_CFG _BV(ISC01)
#define MODEM_CLK_INT_SIG SIG_INTERRUPT0

#define MODEM_OSC_DDR   DDRB
#define MODEM_OSC_PORT  PORTB
#define MODEM_OSC       4
#endif /* CTL_BRD_V1_2 */

#ifdef CTL_BRD_V1_2_1
#define MODEM_CLK_DDR   DDRE
#define MODEM_CLK_PORT  PORTE
#define MODEM_CLK       4
#define MODEM_CLK_INT   INT4
#define MODEM_CLK_INT_REG EICRB
#define MODEM_CLK_INT_CFG _BV(ISC41)
#define MODEM_CLK_INT_SIG SIG_INTERRUPT4
#define MODEM_OSC_DDR   DDRB
#define MODEM_OSC_PORT  PORTB
#define MODEM_OSC       4
#endif /* CTL_BRD_V1_2_1 */



#define MODEM_CHECK_FREE_SPACE(_space) (tx_head>=tx_tail? _space < (TX_BUF_SIZE - (tx_head - tx_tail)) : _space < (tx_tail - tx_head))

#define MODEM_PUT_1_BYTE(_byte) { \
  tx_buf[tx_head] = _byte; \
  tx_head++; \
  if (tx_head >= TX_BUF_SIZE) tx_head = 0; \
}

#define MODEM_PUT_1_BYTE_BY_ADDR(_byte) { \
  tx_buf[tx_head] = *(_byte); \
  ck_a += *(_byte); \
  ck_b += ck_a; \
  tx_head++; \
  if (tx_head >= TX_BUF_SIZE) tx_head = 0; \
}

#define MODEM_PUT_2_BYTE_BY_ADDR(_byte) { \
  MODEM_PUT_1_BYTE_BY_ADDR(_byte); \
  MODEM_PUT_1_BYTE_BY_ADDR(_byte+1); \
}

#define MODEM_PUT_4_BYTE_BY_ADDR(_byte) { \
  MODEM_PUT_1_BYTE_BY_ADDR(_byte); \
  MODEM_PUT_1_BYTE_BY_ADDR(_byte+1); \
  MODEM_PUT_1_BYTE_BY_ADDR(_byte+2); \
  MODEM_PUT_1_BYTE_BY_ADDR(_byte+3); \
}

#define MODEM_LOAD_NEXT_BYTE() { \
  tx_byte = tx_buf[tx_tail]; \
  tx_byte_idx = 0; \
  tx_tail++; \
  if( tx_tail >= TX_BUF_SIZE ) \
    tx_tail = 0; \
}

#define MODEM_CHECK_RUNNING() { \
  if (!(EIMSK & _BV(MODEM_CLK_INT))) { \
    MODEM_LOAD_NEXT_BYTE() \
    sbi(EIFR, INTF0); \
    sbi(EIMSK, MODEM_CLK_INT); \
  } \
}


#endif /* MODEM_H */
