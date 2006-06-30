/*
 * Paparazzi $Id$
 *  
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
 *
 */

#ifndef PPRZ_TRANSPORT_H
#define PPRZ_TRANSPORT_H

#include <inttypes.h>
#include "std.h"
#include "datalink.h"

extern uint8_t ck_a, ck_b;

#define STX  0x99

/** 6 = STX + len + ac_id + msg_id + ck_a + ck_b */
#define PprzTransportSizeOf(_payload) (_payload+6)

#define __Link(dev, _x) dev##_x
#define _Link(dev, _x)  __Link(dev, _x)
#define Link(_x) _Link(DOWNLINK_DEVICE, _x)

#define PprzTransportCheckFreeSpace(_x) Link(CheckFreeSpace(_x))

#define PprzTransportPut1Byte(_x) Link(Transmit(_x))
#define PprzTransportSendMessage() Link(SendMessage())

#define PprzTransportHeader(payload_len) { \
  PprzTransportPut1Byte(STX);				\
  uint8_t msg_len = PprzTransportSizeOf(payload_len);	\
  PprzTransportPut1Byte(msg_len);				\
  ck_a = msg_len; ck_b = msg_len;			\
}

#define PprzTransportTrailer() { \
  PprzTransportPut1Byte(ck_a);	\
  PprzTransportPut1Byte(ck_b);	\
  PprzTransportSendMessage() \
}

#define PprzTransportPutUint8(_byte) { \
    ck_a += _byte;			  \
    ck_b += ck_a;			  \
    PprzTransportPut1Byte(_byte);		  \
 }

#define PprzTransportPutNamedUint8(_name, _byte) PprzTransportPutUint8(_byte)

#define PprzTransportPut1ByteByAddr(_byte) {	 \
    uint8_t _x = *(_byte);		 \
    PprzTransportPutUint8(_x);	 \
  }

#define PprzTransportPut2ByteByAddr(_byte) { \
    PprzTransportPut1ByteByAddr(_byte);	\
    PprzTransportPut1ByteByAddr((const uint8_t*)_byte+1);	\
  }

#define PprzTransportPut4ByteByAddr(_byte) { \
    PprzTransportPut2ByteByAddr(_byte);	\
    PprzTransportPut2ByteByAddr((const uint8_t*)_byte+2);	\
  }


#define PprzTransportPutInt8ByAddr(_x) PprzTransportPut1ByteByAddr(_x)
#define PprzTransportPutUint8ByAddr(_x) PprzTransportPut1ByteByAddr((const uint8_t*)_x)
#define PprzTransportPutInt16ByAddr(_x) PprzTransportPut2ByteByAddr((const uint8_t*)_x)
#define PprzTransportPutUint16ByAddr(_x) PprzTransportPut2ByteByAddr((const uint8_t*)_x)
#define PprzTransportPutInt32ByAddr(_x) PprzTransportPut4ByteByAddr((const uint8_t*)_x)
#define PprzTransportPutUint32ByAddr(_x) PprzTransportPut4ByteByAddr((const uint8_t*)_x)
#define PprzTransportPutFloatByAddr(_x) PprzTransportPut4ByteByAddr((const uint8_t*)_x)

#define PprzTransportPutArray(_put, _n, _x) { \
  uint8_t i; \
  PprzTransportPutUint8(_n); \
  for(i = 0; i < _n; i++) { \
    _put(&_x[i]); \
  } \
}

#define PprzTransportPutFloatArray(_n, _x) PprzTransportPutArray(PprzTransportPutFloatByAddr, _n, _x)
#define PprzTransportPutInt16Array(_n, _x) PprzTransportPutArray(PprzTransportPutInt16ByAddr, _n, _x)

#define PprzTransportPutUint16Array(_n, _x) PprzTransportPutArray(PprzTransportPutUint16ByAddr, _n, _x)
#define PprzTransportPutUint8Array(_n, _x) PprzTransportPutArray(PprzTransportPutUint8ByAddr, _n, _x)


/** Receiving pprz messages */

#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4


#define PPRZ_PAYLOAD_LEN 256
extern uint8_t pprz_payload[PPRZ_PAYLOAD_LEN];

extern volatile bool_t pprz_msg_received;
extern uint8_t pprz_ovrn, pprz_error;
extern volatile uint8_t pprz_payload_len;

static inline void parse_pprz( uint8_t c ) {
  static uint8_t pprz_status = UNINIT;
  static uint8_t _ck_a, _ck_b, payload_idx;

  switch (pprz_status) {
  case UNINIT:
    if (c == STX)
      pprz_status++;
    break;
  case GOT_STX:
    if (pprz_msg_received) {
      pprz_ovrn++;
      goto error;
    }
    pprz_payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    _ck_a = _ck_b = c;
    pprz_status++;
    payload_idx = 0;
    break;
  case GOT_LENGTH:
    pprz_payload[payload_idx] = c;
    _ck_a += c; _ck_b += _ck_a;
    payload_idx++;
    if (payload_idx == pprz_payload_len)
      pprz_status++;
    break;
  case GOT_PAYLOAD:
    if (c != _ck_a)
      goto error;
    pprz_status++;
    break;
  case GOT_CRC1:
    if (c != _ck_b)
      goto error;
    pprz_msg_received = TRUE;
    goto restart;
  }
  return;
 error:
  pprz_error++;
 restart:
  pprz_status = UNINIT;
  return;
}

static inline void pprz_parse_payload(void) {
  uint8_t i;
  for(i = 0; i < pprz_payload_len; i++) 
    dl_buffer[i] = pprz_payload[i];
  dl_msg_available = TRUE;
}

#define __PprzLink(dev, _x) dev##_x
#define _PprzLink(dev, _x)  __PprzLink(dev, _x)
#define PprzLink(_x) _PprzLink(PPRZ_UART, _x)

#define PprzBuffer() PprzLink(ChAvailable())
#define ReadPprzBuffer() { while (PprzLink(ChAvailable())&&!pprz_msg_received) parse_pprz(PprzLink(Getch())); }


#endif /* PPRZ_TRANSPORT_H */

