/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 * @file subsystems/datalink/pprzlog_transport.h
 *
 * Building and Paparazzi frames with timestamp for data logger.
 *
 * LOG-message: ABCDEFGHxxxxxxxI
 *   A PPRZ_STX (0x99)
 *   B LENGTH (H->H)
 *   C SOURCE (0=uart0, 1=uart1, 2=i2c0, ...)
 *   D TIMESTAMP_LSB (100 microsec raster)
 *   E TIMESTAMP
 *   F TIMESTAMP
 *   G TIMESTAMP_MSB
 *   H PPRZ_DATA
 *     0 SENDER_ID
 *     1 MSG_ID
 *     2 MSG_PAYLOAD
 *     . DATA (messages.xml)
 *   I CHECKSUM (sum[B->H])
 *
 */

#ifndef PPRZLOG_TRANSPORT_H
#define PPRZLOG_TRANSPORT_H

#include "mcu_periph/sys_time.h"

extern uint8_t log_ck;

#define STX_LOG  0x99

#define PprzLogTransportSizeOf(_dev, _payload) (_payload)

#define PprzLogTransportCheckFreeSpace(_dev, _x) TransportLink(_dev, CheckFreeSpace(_x))
#define PprzLogTransportPut1Byte(_dev, _x) TransportLink(_dev, Transmit(_x))
#define PprzLogTransportSendMessage(_dev) TransportLink(_dev, SendMessage())

#define PprzLogTransportHeader(_dev, payload_len) {             \
  PprzLogTransportPut1Byte(_dev, STX_LOG);				              \
  uint8_t msg_len = PprzLogTransportSizeOf(_dev, payload_len);	\
  PprzLogTransportPut1Byte(_dev, msg_len);			                \
  log_ck = msg_len;			                                        \
  PprzLogTransportPutUint8(_dev, 0);			                      \
  uint32_t ts = get_sys_time_usec()/100;                        \
  PprzLogTransportPut4ByteByAddr(_dev, &ts);                    \
}

#define PprzLogTransportTrailer(_dev) {  \
  PprzLogTransportPut1Byte(_dev, log_ck);	\
  PprzLogTransportSendMessage(_dev);     \
}

#define PprzLogTransportPutUint8(_dev, _byte) { \
    log_ck += _byte;			  \
    PprzLogTransportPut1Byte(_dev, _byte);		  \
 }

#define PprzLogTransportPutNamedUint8(_dev, _name, _byte) PprzLogTransportPutUint8(_dev, _byte)

#define PprzLogTransportPut1ByteByAddr(_dev, _byte) {	 \
    uint8_t _x = *(_byte);		 \
    PprzLogTransportPutUint8(_dev, _x);	 \
  }

#define PprzLogTransportPut2ByteByAddr(_dev, _byte) { \
    PprzLogTransportPut1ByteByAddr(_dev, _byte);	\
    PprzLogTransportPut1ByteByAddr(_dev, (const uint8_t*)_byte+1);	\
  }

#define PprzLogTransportPut4ByteByAddr(_dev, _byte) { \
    PprzLogTransportPut2ByteByAddr(_dev, _byte);	\
    PprzLogTransportPut2ByteByAddr(_dev, (const uint8_t*)_byte+2);	\
  }

#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
#define PprzLogTransportPutDoubleByAddr(_dev, _byte) { \
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#define PprzLogTransportPutUint64ByAddr(_dev, _byte) { \
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#define PprzLogTransportPutInt64ByAddr(_dev, _byte) { \
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#else
#define PprzLogTransportPutDoubleByAddr(_dev, _byte) { \
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#define PprzLogTransportPutUint64ByAddr(_dev, _byte) { \
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#define PprzLogTransportPutInt64ByAddr(_dev, _byte) { \
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#endif


#define PprzLogTransportPutInt8ByAddr(_dev, _x) PprzLogTransportPut1ByteByAddr(_dev, _x)
#define PprzLogTransportPutUint8ByAddr(_dev, _x) PprzLogTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzLogTransportPutInt16ByAddr(_dev, _x) PprzLogTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzLogTransportPutUint16ByAddr(_dev, _x) PprzLogTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzLogTransportPutInt32ByAddr(_dev, _x) PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzLogTransportPutUint32ByAddr(_dev, _x) PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzLogTransportPutFloatByAddr(_dev, _x) PprzLogTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define PprzLogTransportPutCharByAddr(_dev, _x) PprzLogTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)

#define PprzLogTransportPutArray(_dev, _put, _n, _x) { \
  uint8_t _i; \
  PprzLogTransportPutUint8(_dev, _n); \
  for(_i = 0; _i < _n; _i++) { \
    _put(_dev, &_x[_i]); \
  } \
}

#define PprzLogTransportPutInt8Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutInt8ByAddr, _n, _x)
#define PprzLogTransportPutUint8Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutUint8ByAddr, _n, _x)

#define PprzLogTransportPutCharArray(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutCharByAddr, _n, _x)

#define PprzLogTransportPutInt16Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutInt16ByAddr, _n, _x)
#define PprzLogTransportPutUint16Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutUint16ByAddr, _n, _x)

#define PprzLogTransportPutInt32Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutInt32ByAddr, _n, _x)
#define PprzLogTransportPutUint32Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutUint32ByAddr, _n, _x)

#define PprzLogTransportPutFloatArray(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutFloatByAddr, _n, _x)

#define PprzLogTransportPutInt64Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutInt64ByAddr, _n, _x)
#define PprzLogTransportPutUint64Array(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutUint64ByAddr, _n, _x)

#define PprzLogTransportPutDoubleArray(_dev, _n, _x) PprzLogTransportPutArray(_dev, PprzLogTransportPutDoubleByAddr, _n, _x)

#define PprzLogTransportPutFixedArray(_dev, _put, _n, _x) { \
  uint8_t _i; \
  for(_i = 0; _i < _n; _i++) { \
    _put(_dev, &_x[_i]); \
  } \
}

#define PprzLogTransportPutInt8FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutInt8ByAddr, _n, _x)
#define PprzLogTransportPutUint8FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutUint8ByAddr, _n, _x)

#define PprzLogTransportPutCharFixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutCharByAddr, _n, _x)

#define PprzLogTransportPutInt16FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutInt16ByAddr, _n, _x)
#define PprzLogTransportPutUint16FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutUint16ByAddr, _n, _x)

#define PprzLogTransportPutInt32FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutInt32ByAddr, _n, _x)
#define PprzLogTransportPutUint32FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutUint32ByAddr, _n, _x)

#define PprzLogTransportPutFloatFixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutFloatByAddr, _n, _x)

#define PprzLogTransportPutInt64FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutInt64ByAddr, _n, _x)
#define PprzLogTransportPutUint64FixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutUint64ByAddr, _n, _x)

#define PprzLogTransportPutDoubleFixedArray(_dev, _n, _x) PprzLogTransportPutFixedArray(_dev, PprzLogTransportPutDoubleByAddr, _n, _x)


#endif

