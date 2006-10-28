/*
 * $Id$
 *  
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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

#include "sys_time.h"
#include "print.h"
#include "xbee.h"

#ifdef SIM_UART
#include "sim_uart.h"
#endif

uint8_t xbee_cs;
uint8_t xbee_payload[XBEE_PAYLOAD_LEN];
volatile bool_t xbee_msg_received;
volatile uint8_t xbee_payload_len;
uint8_t xbee_rssi;
uint8_t xbee_ovrn, xbee_error;


#define AT_COMMAND_SEQUENCE "+++"
#define AT_INIT_PERIOD_US 2000000
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_EXIT "ATCN\r"


void xbee_init( void ) {
#ifndef NO_XBEE_API_INIT
  /** Switching to AT mode (FIXME: busy waiting) */
  XBeePrintString(AT_COMMAND_SEQUENCE);

  /** - busy wait 2s FIXME */
  uint8_t init_cpt = 120;
  while (init_cpt) {
    if (sys_time_periodic())
      init_cpt--;
  }


  /** Setting my address */
  XBeePrintString(AT_SET_MY);
  uint16_t addr = XBEE_MY_ADDR;
  XBeePrintHex16(addr);
  XBeePrintString("\r");

  XBeePrintString(AT_AP_MODE);

#ifdef XBEE_INIT
  XBeePrintString(XBEE_INIT);
#endif

  /** Switching back to normal mode */
  XBeePrintString(AT_EXIT);
#endif
}
