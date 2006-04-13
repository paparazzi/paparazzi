/*
 * Paparazzi mcu0 $Id$
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

/* Coronis wavecard serial input and output */

#include <inttypes.h>
#include <stdlib.h>

#include "std.h"
#include "inter_mcu.h"
#include "uart.h"
#include "wavecard.h"
#include "datalink.h"

#define WC_RESET_DDR DDRD
#define WC_RESET_PIN 1
#define WC_RESET_PORT PORTD

void wc_reset( void ) {
  WC_RESET_DDR |= _BV(WC_RESET_PIN);
  sbi(WC_RESET_PORT, WC_RESET_PIN);
}

void wc_end_reset( void ) {
  cbi(WC_RESET_PORT, WC_RESET_PIN);
}


#define WC_MAX_PAYLOAD 256
uint8_t  wc_payload[WC_MAX_PAYLOAD];

static uint8_t wc_status;

const uint16_t poly = 0x8408;
uint16_t crc;
uint8_t wc_length, payload_idx;

#define UNINIT 0
#define GOT_SYNC1 1
#define GOT_STX 2
#define GOT_LENGTH 3
#define GOT_PAYLOAD 4
#define GOT_CRC1 5
#define GOT_CRC2 6


bool_t waiting_ack, wc_msg_received;

uint8_t wc_protocol_error, wc_ovrn, wc_error;


/** Delayed ACK */
SIGNAL(SIG_OUTPUT_COMPARE1B) {
  WcSendAck();
  cbi(TIMSK, OCIE1B);
}

inline void delayed_send_ack( void ) {
  OCR1B = TCNT1 + 1000*CLOCK; /* 1ms delay */
  /* clear interrupt flag  */
  sbi(TIFR, OCF1B);
  /* enable OC1B interrupt */
  sbi(TIMSK, OCIE1B);
}

void wc_parse_payload() {
  switch (wc_payload[0]) {
  case WC_ACK:
    if (waiting_ack)
      waiting_ack = FALSE;
    else
      wc_protocol_error++;
    break;
  case WC_NAK:
  case WC_ERROR:
    wc_protocol_error++;
    break;
  case WC_RECEIVED_FRAME : {
    uint8_t i;
    for(i = 0; i < wc_length-4-WC_ADDR_LEN; i++) 
      dl_buffer[i] = wc_payload[i+WC_ADDR_LEN+1];
    dl_msg_available = TRUE;
    delayed_send_ack();
    break;
  }
    
  default:
    delayed_send_ack();
  }
}

static inline void parse_wc( uint8_t c ) {
  //  printf("s=%d\n", wc_status);
  switch (wc_status) {
  case UNINIT:
    if (c == WC_SYNC)
      wc_status++;
    break;
  case GOT_SYNC1:
    if (c != WC_STX)
      goto error;
    crc = 0;
    wc_status++;
    break;
  case GOT_STX:
    if (wc_msg_received) {
      wc_ovrn++;
      goto error;
    }
    wc_length = c;
    update_crc(c);
    wc_status++;
    payload_idx = 0;
    break;
  case GOT_LENGTH:
    wc_payload[payload_idx] = c;
    update_crc(c);
    payload_idx++;
    if (payload_idx == wc_length-3)
      wc_status++;
    break;
  case GOT_PAYLOAD:
    if (c != (crc & 0xff))
      goto error;
    wc_status++;
    break;
  case GOT_CRC1:
    if (c != (crc >> 8))
      goto error;
    wc_status++;
    break;
  case GOT_CRC2:
    if (c != WC_ETX)
      goto error;
    wc_msg_received = TRUE;
    goto restart;
    break;
  }
  return;
 error:
  wc_error++;
 restart:
  wc_status = UNINIT;
  return;
}

ReceiveUart0(parse_wc);

