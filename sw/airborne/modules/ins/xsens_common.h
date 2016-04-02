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

/** @file modules/ins/xsens_common.h
 * Parser for the Xsens protocol.
 */

#ifndef XSENS_COMMON_H
#define XSENS_COMMON_H

#include "std.h"

/** Includes macros generated from xsens_MTi-G.xml */
#include "xsens_protocol.h"

extern uint8_t xsens_id;
extern uint8_t xsens_status;
extern uint8_t xsens_len;
extern uint8_t xsens_msg_idx;
extern uint8_t ck;
extern uint8_t send_ck;

#define XsensLinkDevice (&((XSENS_LINK).device))

#define XsensInitCheksum() { send_ck = 0; }
#define XsensUpdateChecksum(c) { send_ck += c; }

#define XsensUartSend1(c) XsensLinkDevice->put_byte(XsensLinkDevice->periph, 0, c)
#define XsensSend1(c) { uint8_t i8=c; XsensUartSend1(i8); XsensUpdateChecksum(i8); }
#define XsensSend1ByAddr(x) { XsensSend1(*x); }
#define XsensSend2ByAddr(x) { XsensSend1(*(x+1)); XsensSend1(*x); }
#define XsensSend4ByAddr(x) { XsensSend1(*(x+3)); XsensSend1(*(x+2)); XsensSend1(*(x+1)); XsensSend1(*x); }

#define XsensHeader(msg_id, len) {              \
    XsensUartSend1(XSENS_START);                  \
    XsensInitCheksum();                         \
    XsensSend1(XSENS_BID);                      \
    XsensSend1(msg_id);                         \
    XsensSend1(len);                            \
  }
#define XsensTrailer() { uint8_t i8=0x100-send_ck; XsensUartSend1(i8); }


#define XSENS_MAX_PAYLOAD 254
extern uint8_t xsens_msg_buf[XSENS_MAX_PAYLOAD];

#define UNINIT        0
#define GOT_START     1
#define GOT_BID       2
#define GOT_MID       3
#define GOT_LEN       4
#define GOT_DATA      5
#define GOT_CHECKSUM  6

extern void xsens_event(void);

#endif /* XSENS_COMMON_H */
