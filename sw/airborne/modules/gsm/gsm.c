/*
 * $Id$
 *  
 * Copyright (C) 2009 ENAC
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

#include "gsm.h"
#include "std.h"
#include "downlink.h"

#define GSM_LINK Uart3100
#define GSM_MAX_PAYLOAD 160

#define __GSMLink(dev, _x) dev##_x
#define _GSMLink(dev, _x)  __GSMLink(dev, _x)
#define GSMLink(_x) _GSMLink(GSM_LINK, _x)

#define GSMBuffer() GSMLink(ChAvailable())
#define ReadGSMBuffer() { while (GSMLink(ChAvailable())&&!gsm_line_received) gsm_parse(GSMLink(Getch())); }

static bool gsm_line_received;
static uint8_t gsm_buf[GSM_MAX_PAYLOAD] __attribute__ ((aligned));
static uint8_t gsm_buf_idx;

void gsm_init(void) {
  gsm_buf_idx = 0;
  gsm_line_received = false;
}

void gsm_periodic_1Hz(void) {
  
}

void gsm_send_report(void) {
  
}

void gsm_start(void) {

}

void gsm_stop(void) {

}


static void gsm_parse(uint8_t c) {
  switch(c) {
  case '\n':
    /* ignore */
    break;
  case '\r':
    gsm_line_received = true;
    break;
  default:
    if (gsm_buf_idx < GSM_MAX_PAYLOAD) { 
      gsm_buf[gsm_buf_idx] = c;
      gsm_buf_idx++;
    } /* else extra characters are ignored */
    break;
  }
}

void gsm_event(void) {
  if (GSMBuffer())
    ReadGSMBuffer();

  if (gsm_line_received) {
    /* utiliser l'info */
    gsm_line_received = false;
  }
}

