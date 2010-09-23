/*
 * $Id$
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

/** \file modem_hw.h
 *  \brief AVR CMX469 low level functions
 *
 */


#ifndef MODEM_HW_H
#define MODEM_HW_H

#include <avr/io.h>
#include "std.h"

#define MODEM_CHECK_RUNNING() { \
  if (!(EIMSK & _BV(MODEM_CLK_INT))) { \
    MODEM_LOAD_NEXT_BYTE() \
    sbi(EIFR, INTF0); \
    sbi(EIMSK, MODEM_CLK_INT); \
  } \
}


#define MODEM_TX_PORT   PORTD
#define MODEM_TX_DDR	DDRD
#define MODEM_TX_EN     7
#define MODEM_TX_DATA   6

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

static inline void modem_init ( void ) {
  /* setup TIMER0 to generate a 4MHz clock */
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



#endif
