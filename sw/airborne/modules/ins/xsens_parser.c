/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 */

/** @file modules/ins/xsens_common.c
 * Parser for the Xsens protocol.
 */

#include "xsens_parser.h"

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

uint8_t send_ck;

void xsens_parser_func(struct XsensParser *xsens, uint8_t c);

void xsens_parser_event(struct XsensParser *xsensparser)
{
  struct link_device *dev = &((XSENS_LINK).device);
  if (dev->char_available(dev->periph)) {
    while (dev->char_available(dev->periph) && !xsensparser->msg_received) {
      xsens_parser_func(xsensparser, dev->get_byte(dev->periph));
    }
  }
}

void xsens_parser_func(struct XsensParser *xsens, uint8_t c)
{
  xsens->ck += c;
  switch (xsens->status) {
    case UNINIT:
      if (c != XSENS_START) {
        goto error;
      }
      xsens->status++;
      xsens->ck = 0;
      break;
    case GOT_START:
      if (c != XSENS_BID) {
        goto error;
      }
      xsens->status++;
      break;
    case GOT_BID:
      xsens->id = c;
      xsens->status++;
      break;
    case GOT_MID:
      xsens->len = c;
      if (xsens->len > XSENS_MAX_PAYLOAD) {
        goto error;
      }
      xsens->msg_idx = 0;
      xsens->status++;
      break;
    case GOT_LEN:
      xsens->msg_buf[xsens->msg_idx] = c;
      xsens->msg_idx++;
      if (xsens->msg_idx >= xsens->len) {
        xsens->status++;
      }
      break;
    case GOT_DATA:
      if (xsens->ck != 0) {
        goto error;
      }
      xsens->msg_received = TRUE;

      goto restart;
      break;
    default:
      break;
  }
  return;
error:
restart:
  xsens->status = UNINIT;
  return;
}
