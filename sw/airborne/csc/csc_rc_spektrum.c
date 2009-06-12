/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2009 Joby Energy
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

/** \file csc_rc_spektrum.c
 * \brief Parser for the Spektrum protocol
 */

#include "csc_rc_spektrum.h"

#include <inttypes.h>

#include "led.h"

#include "downlink.h"
#include "messages.h"
#include "uart.h"
#include "string.h"
#include "csc_ap_link.h"
#include "csc_msg_def.h"

#define __SpektrumLink(dev, _x) dev##_x
#define _SpektrumLink(dev, _x)  __SpektrumLink(dev, _x)
#define SpektrumLink(_x) _SpektrumLink(SPEKTRUM_LINK, _x)

#define SPEKTRUM_BUFFER_SIZE  64
static volatile uint8_t msg_received = 0;
static uint8_t msg_buf[SPEKTRUM_BUFFER_SIZE];

#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2

#define SYNC1	      3
#define SYNC2	      1
#define MSG_LENGTH    14

#define bswap_16(x)   ((((x) & 0xFF00) >> 8) | (((x) & 0x00FF) << 8))

void spektrum_init( void )
{
 
}

void spektrum_periodic_task ( void )
{

}

// Called after receipt of valid message
static void spektrum_parse_msg( )
{
  struct spektrum_frame *frame = (struct spektrum_frame *) msg_buf;
  struct CscRCMsg msg; 

  // only copy the channels we need for now to fit within can frame size
  msg.right_stick_vertical = bswap_16(frame->right_vertical);
  msg.right_stick_horizontal = bswap_16(frame->right_horizontal);
  msg.left_stick_horizontal = bswap_16(frame->left_horizontal);
  msg.flap_mix = bswap_16(frame->flap_mix);

  csc_ap_send_msg(CSC_RC_ID, &msg, sizeof(struct CscRCMsg));
}


// Simple state machine parser for spektrum messages
static void parse_spektrum_stream( uint8_t c )
{
  // Parser state
  static uint8_t parser_status = UNINIT;
  static uint8_t msg_idx = 0;

  switch (parser_status) {
  case UNINIT:
    // Look for start byte 1
    if (c != SYNC1)
      parser_status = UNINIT;
    parser_status++; 
    break;
  case GOT_SYNC1:
    // Look for start byte 2
    if (c != SYNC2)
      parser_status = UNINIT;
    parser_status++;
    break;
  case GOT_SYNC2:
    // Save message contents
    msg_buf[msg_idx++] = c;
    if (msg_idx >= MSG_LENGTH) {
      msg_idx = 0;
      parser_status = UNINIT;
      msg_received = TRUE;
    }
    break;
  }
}

void spektrum_event_task( void ) 
{
  while (SpektrumLink(ChAvailable()) && !msg_received) {
    parse_spektrum_stream(SpektrumLink(Getch()));
  }

  if (msg_received) {
    spektrum_parse_msg();
    msg_received = FALSE;
  }
}
