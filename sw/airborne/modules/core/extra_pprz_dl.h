/*
 * Paparazzi $Id: pprz_transport.h 4870 2010-04-24 03:02:39Z poine $
 *
 * Copyright (C) 2010  ENAC
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
 *  \brief Extra datalink using PPRZ protocol
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

#ifndef EXTRA_PPRZ_DL_H
#define EXTRA_PPRZ_DL_H

#include "pprz_transport.h"

#define __ExtraLink(dev, _x) dev##_x
#define _ExtraLink(dev, _x)  __ExtraLink(dev, _x)
#define ExtraLink(_x) _ExtraLink(EXTRA_DOWNLINK_DEVICE, _x)

/** 4 = STX + len + ck_a + ck_b */
#define ExtraPprzTransportSizeOf(_payload) (_payload+4)

#define ExtraPprzTransportCheckFreeSpace(_x) ExtraLink(CheckFreeSpace(_x))

#define ExtraPprzTransportPut1Byte(_x) ExtraLink(Transmit(_x))
#define ExtraPprzTransportSendMessage() ExtraLink(SendMessage())

#define ExtraPprzTransportHeader(payload_len) { \
  ExtraPprzTransportPut1Byte(STX);				\
  uint8_t msg_len = ExtraPprzTransportSizeOf(payload_len);	\
  ExtraPprzTransportPut1Byte(msg_len);			\
  ck_a = msg_len; ck_b = msg_len;			\
}

#define ExtraPprzTransportTrailer() { \
  ExtraPprzTransportPut1Byte(ck_a);	\
  ExtraPprzTransportPut1Byte(ck_b);	\
  ExtraPprzTransportSendMessage() \
}

#define ExtraPprzTransportPutUint8(_byte) { \
    ck_a += _byte;			  \
    ck_b += ck_a;			  \
    ExtraPprzTransportPut1Byte(_byte);		  \
 }

#define ExtraPprzTransportPutNamedUint8(_name, _byte) ExtraPprzTransportPutUint8(_byte)

#define ExtraPprzTransportPut1ByteByAddr(_byte) {	 \
    uint8_t _x = *(_byte);		 \
    ExtraPprzTransportPutUint8(_x);	 \
  }

#define ExtraPprzTransportPut2ByteByAddr(_byte) { \
    ExtraPprzTransportPut1ByteByAddr(_byte);	\
    ExtraPprzTransportPut1ByteByAddr((const uint8_t*)_byte+1);	\
  }

#define ExtraPprzTransportPut4ByteByAddr(_byte) { \
    ExtraPprzTransportPut2ByteByAddr(_byte);	\
    ExtraPprzTransportPut2ByteByAddr((const uint8_t*)_byte+2);	\
  }

#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
#define ExtraPprzTransportPutDoubleByAddr(_byte) { \
    ExtraPprzTransportPut4ByteByAddr((const uint8_t*)_byte+4);	\
    ExtraPprzTransportPut4ByteByAddr((const uint8_t*)_byte);	\
  }
#else
#define ExtraPprzTransportPutDoubleByAddr(_byte) { \
    ExtraPprzTransportPut4ByteByAddr((const uint8_t*)_byte);	\
    ExtraPprzTransportPut4ByteByAddr((const uint8_t*)_byte+4);	\
  }
#endif


#define ExtraPprzTransportPutInt8ByAddr(_x) ExtraPprzTransportPut1ByteByAddr(_x)
#define ExtraPprzTransportPutUint8ByAddr(_x) ExtraPprzTransportPut1ByteByAddr((const uint8_t*)_x)
#define ExtraPprzTransportPutInt16ByAddr(_x) ExtraPprzTransportPut2ByteByAddr((const uint8_t*)_x)
#define ExtraPprzTransportPutUint16ByAddr(_x) ExtraPprzTransportPut2ByteByAddr((const uint8_t*)_x)
#define ExtraPprzTransportPutInt32ByAddr(_x) ExtraPprzTransportPut4ByteByAddr((const uint8_t*)_x)
#define ExtraPprzTransportPutUint32ByAddr(_x) ExtraPprzTransportPut4ByteByAddr((const uint8_t*)_x)
#define ExtraPprzTransportPutFloatByAddr(_x) ExtraPprzTransportPut4ByteByAddr((const uint8_t*)_x)

#define ExtraPprzTransportPutArray(_put, _n, _x) { \
  uint8_t _i; \
  ExtraPprzTransportPutUint8(_n); \
  for(_i = 0; _i < _n; _i++) { \
    _put(&_x[_i]); \
  } \
}

#define ExtraPprzTransportPutFloatArray(_n, _x) ExtraPprzTransportPutArray(ExtraPprzTransportPutFloatByAddr, _n, _x)
#define ExtraPprzTransportPutDoubleArray(_n, _x) ExtraPprzTransportPutArray(ExtraPprzTransportPutDoubleByAddr, _n, _x)

#define ExtraPprzTransportPutInt16Array(_n, _x) ExtraPprzTransportPutArray(ExtraPprzTransportPutInt16ByAddr, _n, _x)
#define ExtraPprzTransportPutUint16Array(_n, _x) ExtraPprzTransportPutArray(ExtraPprzTransportPutUint16ByAddr, _n, _x)

#define ExtraPprzTransportPutInt32Array(_n, _x) ExtraPprzTransportPutArray(ExtraPprzTransportPutInt32ByAddr, _n, _x)
#define ExtraPprzTransportPutUint32Array(_n, _x) ExtraPprzTransportPutArray(ExtraPprzTransportPutUint32ByAddr, _n, _x)

#define ExtraPprzTransportPutUint8Array(_n, _x) ExtraPprzTransportPutArray(ExtraPprzTransportPutUint8ByAddr, _n, _x)


/** Receiving pprz messages */

extern uint8_t extra_pprz_payload[PPRZ_PAYLOAD_LEN];

extern volatile bool_t extra_pprz_msg_received;
extern uint8_t extra_pprz_ovrn, extra_pprz_error;
extern volatile uint8_t extra_pprz_payload_len;

#include "led.h"
//#include "uart.h"
//#include "messages.h"
//#include "downlink.h"
static inline void parse_extra_pprz( uint8_t c ) {
  static uint8_t pprz_status = UNINIT;
  static uint8_t _ck_a, _ck_b, payload_idx;

  //uint8_t tab[] = { c };
  //DOWNLINK_SEND_DEBUG(DefaultChannel,1,tab);
  switch (pprz_status) {
  case UNINIT:
    if (c == STX)
      pprz_status++;
    break;
  case GOT_STX:
    if (extra_pprz_msg_received) {
      extra_pprz_ovrn++;
      goto error;
    }
    extra_pprz_payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    _ck_a = _ck_b = c;
    pprz_status++;
    payload_idx = 0;
    break;
  case GOT_LENGTH:
    extra_pprz_payload[payload_idx] = c;
    _ck_a += c; _ck_b += _ck_a;
    payload_idx++;
    if (payload_idx == extra_pprz_payload_len)
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
    extra_pprz_msg_received = TRUE;
    goto restart;
  default:
    goto error;
  }
  return;
 error:
  extra_pprz_error++;
 restart:
  pprz_status = UNINIT;
  return;
}

static inline void extra_pprz_parse_payload(void) {
  uint8_t i;
  for(i = 0; i < extra_pprz_payload_len; i++)
    dl_buffer[i] = extra_pprz_payload[i];
  dl_msg_available = TRUE;
}

#define __ExtraPprzLink(dev, _x) dev##_x
#define _ExtraPprzLink(dev, _x)  __ExtraPprzLink(dev, _x)
#define ExtraPprzLink(_x) _ExtraPprzLink(EXTRA_PPRZ_UART, _x)

#define ExtraPprzBuffer() ExtraPprzLink(ChAvailable())
#define ReadExtraPprzBuffer() { while (ExtraPprzLink(ChAvailable())&&!extra_pprz_msg_received) parse_extra_pprz(ExtraPprzLink(Getch())); }


/* Datalink Event */

#define ExtraDatalinkEvent() {			\
  if (ExtraPprzBuffer()) {				\
    ReadExtraPprzBuffer();				\
    if (extra_pprz_msg_received) {			\
      extra_pprz_parse_payload();			\
      extra_pprz_msg_received = FALSE;		\
    }						\
  }						\
  if (dl_msg_available) {			\
    dl_parse_msg();				\
    dl_msg_available = FALSE;			\
  }						\
}


#endif /* EXTRA_PPRZ_DL_H */

