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

/** \file pprz_transport.h
 *  \brief Building and parsing Paparazzi frames
 *
 *  Pprz frame:
 *
 *   |STX|length|... payload=(length-4) bytes ...|Checksum A|Checksum B|
 *
 *   where checksum is computed over length and payload:
 *     ck_A = ck_B = length
 *     for each byte b in payload
 *       ck_A += b; ck_b += ck_A
 */

#ifndef PPRZ_TRANSPORT_H
#define PPRZ_TRANSPORT_H

#include <inttypes.h>
#include "std.h"
#include "subsystems/datalink/datalink.h"

extern uint8_t ck_a, ck_b;

#define STX  0x99

/** 4 = STX + len + ck_a + ck_b */
#define PprzTransportSizeOf(_payload) (_payload+4)

#define __Link(dev, _x) dev##_x
#define _Link(dev, _x)  __Link(dev, _x)
#define Link(_dev, _x) _Link(_dev, _x)

#define PprzTransportCheckFreeSpace(_dev, _x) Link(_dev, CheckFreeSpace(_x))

#define PprzTransportPut1Byte(_dev, _x) Link(_dev, Transmit(_x))
#define PprzTransportSendMessage(_dev) Link(_dev, SendMessage())

#define PprzTransportHeader(_dev, payload_len) { \
  PprzTransportPut1Byte(_dev, STX);				\
  uint8_t msg_len = PprzTransportSizeOf(payload_len);	\
  PprzTransportPut1Byte(_dev, msg_len);			\
  ck_a = msg_len; ck_b = msg_len;			\
}

#define PprzTransportTrailer(_dev) { \
  PprzTransportPut1Byte(_dev, ck_a);	\
  PprzTransportPut1Byte(_dev, ck_b);	\
  PprzTransportSendMessage(_dev) \
}

#define PprzTransportPutUint8(_dev, _byte) { \
    ck_a += _byte;			  \
    ck_b += ck_a;			  \
    PprzTransportPut1Byte(_dev, _byte);		  \
 }

#define PprzTransportPutNamedUint8(_dev, _name, _byte) PprzTransportPutUint8(_dev, _byte)

#define PprzTransportPut1ByteByAddr(_dev, _byte) {	 \
    uint8_t _x = *(_byte);		 \
    PprzTransportPutUint8(_dev, _x);	 \
  }

#define PprzTransportPut2ByteByAddr(_dev, _byte) { \
    PprzTransportPut1ByteByAddr(_dev, _byte);	\
    PprzTransportPut1ByteByAddr(_dev, (const uint8_t*)_byte+1);	\
  }

#define PprzTransportPut4ByteByAddr(_dev, _byte) { \
    PprzTransportPut2ByteByAddr(_dev, _byte);	\
    PprzTransportPut2ByteByAddr(_dev, (const uint8_t*)_byte+2);	\
  }

#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
#define PprzTransportPutDoubleByAddr(_dev, _byte) { \
    PprzTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    PprzTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#else
#define PprzTransportPutDoubleByAddr(_dev, _byte) { \
    PprzTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    PprzTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#endif


#define PprzTransportPutInt8ByAddr(_dev, _x) PprzTransportPut1ByteByAddr(_dev, _x)
#define PprzTransportPutUint8ByAddr(_dev, _x) PprzTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzTransportPutInt16ByAddr(_dev, _x) PprzTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzTransportPutUint16ByAddr(_dev, _x) PprzTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzTransportPutInt32ByAddr(_dev, _x) PprzTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzTransportPutUint32ByAddr(_dev, _x) PprzTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzTransportPutFloatByAddr(_dev, _x) PprzTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)

#define PprzTransportPutArray(_dev, _put, _n, _x) { \
  uint8_t _i; \
  PprzTransportPutUint8(_dev, _n); \
  for(_i = 0; _i < _n; _i++) { \
    _put(_dev, &_x[_i]); \
  } \
}

#define PprzTransportPutFloatArray(_dev, _n, _x) PprzTransportPutArray(_dev, PprzTransportPutFloatByAddr, _n, _x)
#define PprzTransportPutDoubleArray(_dev, _n, _x) PprzTransportPutArray(_dev, PprzTransportPutDoubleByAddr, _n, _x)

#define PprzTransportPutInt16Array(_dev, _n, _x) PprzTransportPutArray(_dev, PprzTransportPutInt16ByAddr, _n, _x)
#define PprzTransportPutUint16Array(_dev, _n, _x) PprzTransportPutArray(_dev, PprzTransportPutUint16ByAddr, _n, _x)

#define PprzTransportPutInt32Array(_dev, _n, _x) PprzTransportPutArray(_dev, PprzTransportPutInt32ByAddr, _n, _x)
#define PprzTransportPutUint32Array(_dev, _n, _x) PprzTransportPutArray(_dev, PprzTransportPutUint32ByAddr, _n, _x)

#define PprzTransportPutUint8Array(_dev, _n, _x) PprzTransportPutArray(_dev, PprzTransportPutUint8ByAddr, _n, _x)


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
  default:
    goto error;
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

