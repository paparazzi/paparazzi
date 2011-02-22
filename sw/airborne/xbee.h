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

/* Maxstream XBee serial input and output */

#ifndef XBEE_H
#define XBEE_H

#include "datalink.h"
#include "generated/airframe.h"

#ifdef XBEE868
#include "xbee868.h"
#else /* Not 868 */
#include "xbee24.h"
#endif

/** Constants for the API protocol */
#define XBEE_START 0x7e
#define TX_OPTIONS 0x00
#define NO_FRAME_ID 0

#define GROUND_STATION_ADDR 0x100

extern uint8_t xbee_cs;
#define XBEE_PAYLOAD_LEN 256
extern uint8_t xbee_payload[XBEE_PAYLOAD_LEN];

extern volatile bool_t xbee_msg_received;
extern volatile uint8_t xbee_payload_len;
extern uint8_t xbee_rssi;
extern uint8_t xbee_ovrn, xbee_error;

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
#define XBEE_MY_ADDR AC_ID
void xbee_init( void );

#define __Link(dev, _x) dev##_x
#define _Link(dev, _x)  __Link(dev, _x)
#define Link(_x) _Link(DOWNLINK_DEVICE, _x)

#define XBeeTransportPut1Byte(x) Link(Transmit(x))
#define XBeeTransportCheckFreeSpace(x) Link(CheckFreeSpace(x))
/* 5 = Start + len_msb + len_lsb + API_id + checksum */
#define XBeeAPISizeOf(_x) (_x+5)
#define XBeeTransportSendMessage() Link(SendMessage())

#define XBeeTransportPutUint8(_x) { \
  xbee_cs += _x; \
  XBeeTransportPut1Byte(_x); \
}

#define XBeeTransportPut1ByteByAddr(_byte) { \
  uint8_t _x = *(_byte);	\
  XBeeTransportPutUint8(_x);	 \
 }

#define XBeeTransportPut2Bytes(_x) { \
  uint16_t x16 = _x; \
  XBeeTransportPut1Byte(x16>>8); \
  XBeeTransportPut1Byte(x16 & 0xff); \
}

#define XBeeTransportPut2ByteByAddr(_byte) { \
    XBeeTransportPut1ByteByAddr(_byte);	\
    XBeeTransportPut1ByteByAddr((const uint8_t*)_byte+1);	\
  }

#define XBeeTransportPut4ByteByAddr(_byte) { \
    XBeeTransportPut2ByteByAddr(_byte);	\
    XBeeTransportPut2ByteByAddr((const uint8_t*)_byte+2); \
  }

#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
#define XBeeTransportPutDoubleByAddr(_byte) { \
    XBeeTransportPut4ByteByAddr((const uint8_t*)_byte+4);	\
    XBeeTransportPut4ByteByAddr((const uint8_t*)_byte);	\
  }
#else
#define XBeeTransportPutDoubleByAddr(_byte) { \
    XBeeTransportPut4ByteByAddr((const uint8_t*)_byte);	\
    XBeeTransportPut4ByteByAddr((const uint8_t*)_byte+4);	\
  }
#endif


#define XBeeTransportPutInt8ByAddr(_x) XBeeTransportPut1ByteByAddr(_x)
#define XBeeTransportPutUint8ByAddr(_x) XBeeTransportPut1ByteByAddr((const uint8_t*)_x)
#define XBeeTransportPutInt16ByAddr(_x) XBeeTransportPut2ByteByAddr((const uint8_t*)_x)
#define XBeeTransportPutUint16ByAddr(_x) XBeeTransportPut2ByteByAddr((const uint8_t*)_x)
#define XBeeTransportPutInt32ByAddr(_x) XBeeTransportPut4ByteByAddr((const uint8_t*)_x)
#define XBeeTransportPutUint32ByAddr(_x) XBeeTransportPut4ByteByAddr((const uint8_t*)_x)
#define XBeeTransportPutFloatByAddr(_x) XBeeTransportPut4ByteByAddr((const uint8_t*)_x)
#define XBeeTransportPutNamedUint8(_name, _byte) XBeeTransportPutUint8(_byte)

#define XBeeTransportPutArray(_put, _n, _x) { \
  uint8_t _i; \
  XBeeTransportPutUint8(_n); \
  for(_i = 0; _i < _n; _i++) { \
    _put(&_x[_i]); \
  } \
}

#define XBeeTransportPutInt16Array(_n, _x) XBeeTransportPutArray(XBeeTransportPutInt16ByAddr, _n, _x)

#define XBeeTransportPutUint16Array(_n, _x) XBeeTransportPutArray(XBeeTransportPutUint16ByAddr, _n, _x)
#define XBeeTransportPutUint8Array(_n, _x) XBeeTransportPutArray(XBeeTransportPutUint8ByAddr, _n, _x)
#define XBeeTransportPutFloatArray(_n, _x) XBeeTransportPutArray(XBeeTransportPutFloatByAddr, _n, _x)
#define XBeeTransportPutDoubleArray(_n, _x) XBeeTransportPutArray(XBeeTransportPutDoubleByAddr, _n, _x)



#define XBeeTransportHeader(_len) { \
  XBeeTransportPut1Byte(XBEE_START); \
  uint8_t payload_len = XBeeAPISizeOf(_len); \
  XBeeTransportPut2Bytes(payload_len); \
  xbee_cs = 0; \
  XBeeTransportPutTXHeader(); \
}

#define XBeeTransportTrailer() { \
  xbee_cs = 0xff - xbee_cs; \
  XBeeTransportPut1Byte(xbee_cs); \
  XBeeTransportSendMessage() \
}



/** Status of the API packet receiver automata */
#define XBEE_UNINIT 0
#define XBEE_GOT_START 1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD 4


/** Parsing a XBee API frame */
static inline void parse_xbee( uint8_t c ) {
  static uint8_t xbee_status = XBEE_UNINIT;
  static uint8_t cs, payload_idx;

  switch (xbee_status) {
  case XBEE_UNINIT:
    if (c == XBEE_START)
      xbee_status++;
    break;
  case XBEE_GOT_START:
    if (xbee_msg_received) {
      xbee_ovrn++;
      goto error;
    }
    xbee_payload_len = c<<8;
    xbee_status++;
    break;
  case XBEE_GOT_LENGTH_MSB:
    xbee_payload_len |= c;
    xbee_status++;
    payload_idx = 0;
    cs=0;
    break;
  case XBEE_GOT_LENGTH_LSB:
    xbee_payload[payload_idx] = c;
    cs += c;
    payload_idx++;
    if (payload_idx == xbee_payload_len)
      xbee_status++;
    break;
  case XBEE_GOT_PAYLOAD:
    if (c + cs != 0xff)
      goto error;
    xbee_msg_received = TRUE;
    goto restart;
    break;
  default:
    goto error;
  }
  return;
 error:
  xbee_error++;
 restart:
  xbee_status = XBEE_UNINIT;
  return;
}

/** Parsing a frame data and copy the payload to the datalink buffer */
static inline void xbee_parse_payload(void) {
  switch (xbee_payload[0]) {
  case XBEE_RX_ID:
  case XBEE_TX_ID: /* Useful if A/C is connected to the PC with a cable */
    XbeeGetRSSI();
    uint8_t i;
    for(i = XBEE_RFDATA_OFFSET; i < xbee_payload_len; i++)
      dl_buffer[i-XBEE_RFDATA_OFFSET] = xbee_payload[i];
    dl_msg_available = TRUE;
    break;
  default:
    return;
  }
}

#define __XBeeLink(dev, _x) dev##_x
#define _XBeeLink(dev, _x)  __XBeeLink(dev, _x)
#define XBeeLink(_x) _XBeeLink(XBEE_UART, _x)

#define XBeeBuffer() XBeeLink(ChAvailable())
#define ReadXBeeBuffer() { while (XBeeLink(ChAvailable())&&!xbee_msg_received) parse_xbee(XBeeLink(Getch())); }

#define XBeePrintString(s) XBeeLink(PrintString(s))
#define XBeePrintHex16(x) XBeeLink(PrintHex16(x))

#endif /* XBEE_H */
