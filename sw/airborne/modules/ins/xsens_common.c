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

#include "xsens_common.h"

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

volatile uint8_t xsens_msg_received;

uint8_t xsens_id;
uint8_t xsens_status;
uint8_t xsens_len;
uint8_t xsens_msg_idx;
uint8_t ck;
uint8_t send_ck;

uint8_t xsens_msg_buf[XSENS_MAX_PAYLOAD];

void parse_xsens_buffer(uint8_t c);

void xsens_event(void)
{
  struct link_device *dev = &((XSENS_LINK).device);
  if (dev->char_available(dev->periph)) {
    while (dev->char_available(dev->periph) && !xsens_msg_received) {
      parse_xsens_buffer(dev->get_byte(dev->periph));
    }
  }
}

void parse_xsens_buffer(uint8_t c)
{
  ck += c;
  switch (xsens_status) {
    case UNINIT:
      if (c != XSENS_START) {
        goto error;
      }
      xsens_status++;
      ck = 0;
      break;
    case GOT_START:
      if (c != XSENS_BID) {
        goto error;
      }
      xsens_status++;
      break;
    case GOT_BID:
      xsens_id = c;
      xsens_status++;
      break;
    case GOT_MID:
      xsens_len = c;
      if (xsens_len > XSENS_MAX_PAYLOAD) {
        goto error;
      }
      xsens_msg_idx = 0;
      xsens_status++;
      break;
    case GOT_LEN:
      xsens_msg_buf[xsens_msg_idx] = c;
      xsens_msg_idx++;
      if (xsens_msg_idx >= xsens_len) {
        xsens_status++;
      }
      break;
    case GOT_DATA:
      if (ck != 0) {
        goto error;
      }
      xsens_msg_received = TRUE;
      goto restart;
      break;
    default:
      break;
  }
  return;
error:
restart:
  xsens_status = UNINIT;
  return;
}
