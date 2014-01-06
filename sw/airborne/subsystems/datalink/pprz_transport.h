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
 *
 */

/**
 * @file subsystems/datalink/pprz_transport.h
 *
 * Building and parsing Paparazzi frames.
 *
 * Pprz frame:
 *
 * |STX|length|... payload=(length-4) bytes ...|Checksum A|Checksum B|
 *
 * where checksum is computed over length and payload:
 * @code
 * ck_A = ck_B = length
 * for each byte b in payload
 *     ck_A += b;
 *     ck_b += ck_A;
 * @endcode
 */

#ifndef PPRZ_TRANSPORT_H
#define PPRZ_TRANSPORT_H

#include <inttypes.h>
#include "std.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/transport.h"

/* PPRZ Transport
 * downlink macros
 */
extern uint8_t ck_a, ck_b;

#define STX  0x99

/** 4 = STX + len + ck_a + ck_b */
#define PprzTransportSizeOf(_dev, _payload) (_payload+4)

#define PprzTransportCheckFreeSpace(_dev, _x) TransportLink(_dev, CheckFreeSpace(_x))
#define PprzTransportPut1Byte(_dev, _x) TransportLink(_dev, Transmit(_x))
#define PprzTransportSendMessage(_dev) TransportLink(_dev, SendMessage())

#define PprzTransportHeader(_dev, payload_len) { \
  PprzTransportPut1Byte(_dev, STX);				\
  uint8_t msg_len = PprzTransportSizeOf(_dev, payload_len);	\
  PprzTransportPut1Byte(_dev, msg_len);			\
  ck_a = msg_len; ck_b = msg_len;			\
}

#define PprzTransportTrailer(_dev) { \
  PprzTransportPut1Byte(_dev, ck_a);	\
  PprzTransportPut1Byte(_dev, ck_b);	\
  PprzTransportSendMessage(_dev); \
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

// PPRZ parsing state machine
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_PAYLOAD 3
#define GOT_CRC1    4

struct pprz_transport {
  // generic interface
  struct transport trans;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};

extern struct pprz_transport pprz_tp;

static inline void parse_pprz(struct pprz_transport * t, uint8_t c ) {
  switch (t->status) {
  case UNINIT:
    if (c == STX)
      t->status++;
    break;
  case GOT_STX:
    if (t->trans.msg_received) {
      t->trans.ovrn++;
      goto error;
    }
    t->trans.payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    t->ck_a = t->ck_b = c;
    t->status++;
    t->payload_idx = 0;
    break;
  case GOT_LENGTH:
    t->trans.payload[t->payload_idx] = c;
    t->ck_a += c; t->ck_b += t->ck_a;
    t->payload_idx++;
    if (t->payload_idx == t->trans.payload_len)
      t->status++;
    break;
  case GOT_PAYLOAD:
    if (c != t->ck_a)
      goto error;
    t->status++;
    break;
  case GOT_CRC1:
    if (c != t->ck_b)
      goto error;
    t->trans.msg_received = TRUE;
    goto restart;
  default:
    goto error;
  }
  return;
 error:
  t->trans.error++;
 restart:
  t->status = UNINIT;
  return;
}

static inline void pprz_parse_payload(struct pprz_transport * t) {
  uint8_t i;
  for(i = 0; i < t->trans.payload_len; i++)
    dl_buffer[i] = t->trans.payload[i];
  dl_msg_available = TRUE;
}


#define PprzBuffer(_dev) TransportLink(_dev,ChAvailable())
#define ReadPprzBuffer(_dev,_trans) { while (TransportLink(_dev,ChAvailable())&&!(_trans.trans.msg_received)) parse_pprz(&(_trans),TransportLink(_dev,Getch())); }
#define PprzCheckAndParse(_dev,_trans) {  \
  if (PprzBuffer(_dev)) {                 \
    ReadPprzBuffer(_dev,_trans);          \
    if (_trans.trans.msg_received) {      \
      pprz_parse_payload(&(_trans));      \
      _trans.trans.msg_received = FALSE;  \
    }                                     \
  }                                       \
}


#endif /* PPRZ_TRANSPORT_H */

