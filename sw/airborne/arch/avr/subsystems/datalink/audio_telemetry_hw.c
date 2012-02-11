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

/** \file audio_telemetry_hw.c
 *  \brief Handling of a CMX 469 on avr mega128 architecture
 */

#include <inttypes.h>

#if (__GNUC__ == 3)
#include <avr/signal.h>
#endif

#include <avr/interrupt.h>
#include "audio_telemetry.h"

uint8_t audio_telemetry_nb_ovrn;

uint8_t           tx_head;
volatile uint8_t  tx_tail;
uint8_t           tx_buf[ TX_BUF_SIZE ];

uint8_t    tx_byte;
uint8_t    tx_byte_idx;


SIGNAL( AUDIO_TELEMETRY_CLK_INT_SIG ) {
  /*  start bit         */
  if (tx_byte_idx == 0)
    cbi(AUDIO_TELEMETRY_TX_PORT, AUDIO_TELEMETRY_TX_DATA);
  /* 8 data bits        */
  else if (tx_byte_idx < 9) {
    if (tx_byte & 0x01)
      sbi(AUDIO_TELEMETRY_TX_PORT, AUDIO_TELEMETRY_TX_DATA);
    else
      cbi(AUDIO_TELEMETRY_TX_PORT, AUDIO_TELEMETRY_TX_DATA);
    tx_byte >>= 1;
  }
  /* stop_bit           */
  else {
    sbi(AUDIO_TELEMETRY_TX_PORT, AUDIO_TELEMETRY_TX_DATA);
  }
  tx_byte_idx++;
  /* next byte          */
  if (tx_byte_idx >= 10) {
    /*  if we have nothing left to transmit */
    if( tx_head == tx_tail ) {
      /* disable clock interrupt            */
      cbi( EIMSK, AUDIO_TELEMETRY_CLK_INT );
    } else {
      /* else load next byte                  */
      AUDIO_TELEMETRY_LOAD_NEXT_BYTE();
    }
  }
}
