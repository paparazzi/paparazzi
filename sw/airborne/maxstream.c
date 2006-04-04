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

/* Maxstream input and output */

#include <inttypes.h>
#include <stdlib.h>

#include "maxstream.h"
#include "messages.h"
#include "datalink.h"

static uint8_t msg_lengths[] = MSG_LENGTHS;

#define PprzMsgLength(c) (c<32 ? msg_lengths[c] : 0)


#define UNINIT 0
#define GOT_STX 1
#define GOT_ID 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4
#define GOT_CRC2 5


bool_t maxstream_msg_received;

uint8_t maxstream_ovrn, maxstream_error;

static uint8_t payload_length;

#define MAXSTREAM_MAX_PAYLOAD 256
static uint8_t payload[MAXSTREAM_MAX_PAYLOAD];


void maxstream_parse_payload(void) {
  uint8_t i;
  for(i = 0; i < payload_length; i++) 
    dl_buffer[i] = payload[i];
  dl_msg_available = TRUE;
}

#define MAXSTREAM_STX 0x05 /** == modem.h:STX */


static inline void parse_maxstream( uint8_t c ) {
  static uint8_t maxstream_status = UNINIT;
  static uint8_t ck_a, ck_b, payload_idx;

  switch (maxstream_status) {
  case UNINIT:
    if (c == MAXSTREAM_STX)
      maxstream_status++;
    break;
  case GOT_STX:
    /** Beginning of the payload; first char is the id of the message */
    if (maxstream_msg_received) {
      maxstream_ovrn++;
      goto error;
    }
    payload_length = PprzMsgLength(c); /* Not including STX, CRC1 and CRC2 */
    if (!payload_length)
      goto error;
    payload[0] = c;
    ck_a = ck_b = c;
    maxstream_status++;
    payload_idx = 1;
    break;
  case GOT_ID:
    payload[payload_idx] = c;
    ck_a += c; ck_b += ck_a;
    payload_idx++;
    if (payload_idx == payload_length)
      maxstream_status++;
    break;
  case GOT_PAYLOAD:
    if (c != ck_a)
      goto error;
    maxstream_status++;
    break;
  case GOT_CRC1:
    if (c != ck_b)
      goto error;
    maxstream_msg_received = TRUE;
    goto restart;
  }
  return;
 error:
  maxstream_error++;
 restart:
  maxstream_status = UNINIT;
  return;
}

ReceiveUart0(parse_maxstream);

