/*
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

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"

#ifdef XBEE868
#include "subsystems/datalink/xbee868.h"
#else /* Not 868 */
#include "subsystems/datalink/xbee24.h"
#endif
#include "subsystems/datalink/transport.h"

/** Constants for the API protocol */
#define XBEE_START 0x7e
#define TX_OPTIONS 0x00
#define NO_FRAME_ID 0

/** Ground station address */
#define GROUND_STATION_ADDR 0x100

extern uint8_t xbee_cs;
extern uint8_t xbee_rssi;

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
#define XBEE_MY_ADDR AC_ID
void xbee_init( void );

/* 5 = Start + len_msb + len_lsb + API_id + checksum */
#define XBeeAPISizeOf(_dev, _x) (_x+5)

#define XBeeTransportCheckFreeSpace(_dev, x) TransportLink(_dev, CheckFreeSpace(x))
#define XBeeTransportPut1Byte(_dev, x) TransportLink(_dev, Transmit(x))
#define XBeeTransportSendMessage(_dev) TransportLink(_dev, SendMessage())

#define XBeeTransportPutUint8(_dev, _x) { \
  xbee_cs += _x; \
  XBeeTransportPut1Byte(_dev, _x); \
}

#define XBeeTransportPut1ByteByAddr(_dev, _byte) { \
  uint8_t _x = *(_byte);	\
  XBeeTransportPutUint8(_dev, _x);	 \
 }

#define XBeeTransportPut2Bytes(_dev, _x) { \
  uint16_t x16 = _x; \
  XBeeTransportPut1Byte(_dev, x16>>8); \
  XBeeTransportPut1Byte(_dev, x16 & 0xff); \
}

#define XBeeTransportPut2ByteByAddr(_dev, _byte) { \
    XBeeTransportPut1ByteByAddr(_dev, _byte);	\
    XBeeTransportPut1ByteByAddr(_dev, (const uint8_t*)_byte+1);	\
  }

#define XBeeTransportPut4ByteByAddr(_dev, _byte) { \
    XBeeTransportPut2ByteByAddr(_dev, _byte);	\
    XBeeTransportPut2ByteByAddr(_dev, (const uint8_t*)_byte+2); \
  }

#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
#define XBeeTransportPutDoubleByAddr(_dev, _byte) { \
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#define XBeeTransportPutUint64ByAddr(_dev, _byte) { \
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#define XBeeTransportPutInt64ByAddr(_dev, _byte) { \
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#else
#define XBeeTransportPutDoubleByAddr(_dev, _byte) { \
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#define XBeeTransportPutUint64ByAddr(_dev, _byte) { \
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#define XBeeTransportPutInt64ByAddr(_dev, _byte) { \
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#endif


#define XBeeTransportPutInt8ByAddr(_dev, _x) XBeeTransportPut1ByteByAddr(_dev, _x)
#define XBeeTransportPutUint8ByAddr(_dev, _x) XBeeTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)
#define XBeeTransportPutInt16ByAddr(_dev, _x) XBeeTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define XBeeTransportPutUint16ByAddr(_dev, _x) XBeeTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define XBeeTransportPutInt32ByAddr(_dev, _x) XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define XBeeTransportPutUint32ByAddr(_dev, _x) XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define XBeeTransportPutFloatByAddr(_dev, _x) XBeeTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define XBeeTransportPutNamedUint8(_dev, _name, _byte) XBeeTransportPutUint8(_dev, _byte)
#define XBeeTransportPutCharByAddr(_dev, _x) XBeeTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)

#define XBeeTransportPutArray(_dev, _put, _n, _x) { \
  uint8_t _i; \
  XBeeTransportPutUint8(_dev, _n); \
  for(_i = 0; _i < _n; _i++) { \
    _put(_dev, &_x[_i]); \
  } \
}

#define XBeeTransportPutInt8Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutInt8ByAddr, _n, _x)
#define XBeeTransportPutUint8Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutUint8ByAddr, _n, _x)

#define XBeeTransportPutCharArray(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutCharByAddr, _n, _x)

#define XBeeTransportPutInt16Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutInt16ByAddr, _n, _x)
#define XBeeTransportPutUint16Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutUint16ByAddr, _n, _x)

#define XBeeTransportPutInt32Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutInt32ByAddr, _n, _x)
#define XBeeTransportPutUint32Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutUint32ByAddr, _n, _x)

#define XBeeTransportPutFloatArray(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutFloatByAddr, _n, _x)

#define XBeeTransportPutInt64Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutInt64ByAddr, _n, _x)
#define XBeeTransportPutUint64Array(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutUint64ByAddr, _n, _x)

#define XBeeTransportPutDoubleArray(_dev, _n, _x) XBeeTransportPutArray(_dev, XBeeTransportPutDoubleByAddr, _n, _x)


#define XBeeTransportPutFixedArray(_dev, _put, _n, _x) { \
  uint8_t _i; \
  for(_i = 0; _i < _n; _i++) { \
    _put(_dev, &_x[_i]); \
  } \
}

#define XBeeTransportPutInt8FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutInt8ByAddr, _n, _x)
#define XBeeTransportPutUint8FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutUint8ByAddr, _n, _x)

#define XBeeTransportPutCharFixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutCharByAddr, _n, _x)

#define XBeeTransportPutInt16FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutInt16ByAddr, _n, _x)
#define XBeeTransportPutUint16FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutUint16ByAddr, _n, _x)

#define XBeeTransportPutInt32FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutInt32ByAddr, _n, _x)
#define XBeeTransportPutUint32FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutUint32ByAddr, _n, _x)

#define XBeeTransportPutFloatFixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutFloatByAddr, _n, _x)

#define XBeeTransportPutInt64FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutInt64ByAddr, _n, _x)
#define XBeeTransportPutUint64FixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutUint64ByAddr, _n, _x)

#define XBeeTransportPutDoubleFixedArray(_dev, _n, _x) XBeeTransportPutFixedArray(_dev, XBeeTransportPutDoubleByAddr, _n, _x)


#define XBeeTransportHeader(_dev, _len) { \
  XBeeTransportPut1Byte(_dev, XBEE_START); \
  uint8_t payload_len = XBeeAPISizeOf(_dev, _len); \
  XBeeTransportPut2Bytes(_dev, payload_len); \
  xbee_cs = 0; \
  XBeeTransportPutTXHeader(_dev); \
}

#define XBeeTransportTrailer(_dev) { \
  xbee_cs = 0xff - xbee_cs; \
  XBeeTransportPut1Byte(_dev, xbee_cs); \
  XBeeTransportSendMessage(_dev) \
}



/** Status of the API packet receiver automata */
#define XBEE_UNINIT         0
#define XBEE_GOT_START      1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD    4

struct xbee_transport {
  // generic interface
  struct transport trans;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t cs;
};

extern struct xbee_transport xbee_tp;

/** Parsing a XBee API frame */
static inline void parse_xbee( struct xbee_transport * t, uint8_t c ) {
  switch (t->status) {
  case XBEE_UNINIT:
    if (c == XBEE_START)
      t->status++;
    break;
  case XBEE_GOT_START:
    if (t->trans.msg_received) {
      t->trans.ovrn++;
      goto error;
    }
    t->trans.payload_len = c<<8;
    t->status++;
    break;
  case XBEE_GOT_LENGTH_MSB:
    t->trans.payload_len |= c;
    t->status++;
    t->payload_idx = 0;
    t->cs=0;
    break;
  case XBEE_GOT_LENGTH_LSB:
    t->trans.payload[t->payload_idx] = c;
    t->cs += c;
    t->payload_idx++;
    if (t->payload_idx == t->trans.payload_len)
      t->status++;
    break;
  case XBEE_GOT_PAYLOAD:
    if (c + t->cs != 0xff)
      goto error;
    t->trans.msg_received = TRUE;
    goto restart;
    break;
  default:
    goto error;
  }
  return;
 error:
  t->trans.error++;
 restart:
  t->status = XBEE_UNINIT;
  return;
}

/** Parsing a frame data and copy the payload to the datalink buffer */
static inline void xbee_parse_payload(struct xbee_transport * t) {
  switch (t->trans.payload[0]) {
  case XBEE_RX_ID:
  case XBEE_TX_ID: /* Useful if A/C is connected to the PC with a cable */
    XbeeGetRSSI(t->trans.payload);
    uint8_t i;
    for(i = XBEE_RFDATA_OFFSET; i < t->trans.payload_len; i++)
      dl_buffer[i-XBEE_RFDATA_OFFSET] = t->trans.payload[i];
    dl_msg_available = TRUE;
    break;
  default:
    return;
  }
}

#define XBeeBuffer(_dev) TransportLink(_dev,ChAvailable())
#define ReadXBeeBuffer(_dev,_trans) { while (TransportLink(_dev,ChAvailable())&&!(_trans.trans.msg_received)) parse_xbee(&(_trans),TransportLink(_dev,Getch())); }
#define XBeeCheckAndParse(_dev,_trans) {  \
  if (XBeeBuffer(_dev)) {                 \
    ReadXBeeBuffer(_dev,_trans);          \
    if (_trans.trans.msg_received) {      \
      xbee_parse_payload(&(_trans));      \
      _trans.trans.msg_received = FALSE;  \
    }                                     \
  }                                       \
}

#define XBeePrintString(_dev, s) TransportLink(_dev,PrintString(s))
#define XBeePrintHex16(_dev, x) TransportLink(_dev,PrintHex16(x))

#endif /* XBEE_H */
