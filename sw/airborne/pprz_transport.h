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

extern uint8_t ck_a, ck_b;

#define STX  0x99

/** 6 = STX + len + ac_id + msg_id + ck_a + ck_b */
#define PprzTransportSizeOf(_payload) (_payload+6)

#define __Link(dev, _x) dev##_x
#define _Link(dev, _x)  __Link(dev, _x)
#define Link(_x) _Link(DOWNLINK_DEVICE, _x)

#define PprzTransportCheckFreeSpace(_x) Link(_check_free_space(_x))

#define PprzTransportPut1Byte(_x) Link(_transmit(_x))

#define PprzTransportHeader(payload_len) { \
  PprzTransportPut1Byte(STX);				\
  uint8_t msg_len = PprzTransportSizeOf(payload_len);	\
  PprzTransportPut1Byte(msg_len);				\
  ck_a = msg_len; ck_b = msg_len;			\
}

#define PprzTransportTrailer() { \
  PprzTransportPut1Byte(ck_a);			\
  PprzTransportPut1Byte(ck_b);			\
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
    PprzTransportPut1ByteByAddr(_byte+1);	\
  }

#define PprzTransportPut4ByteByAddr(_byte) { \
    PprzTransportPut2ByteByAddr(_byte);	\
    PprzTransportPut2ByteByAddr(_byte+2);	\
  }


#define PprzTransportPutInt8ByAddr(_x) PprzTransportPut1ByteByAddr(_x)
#define PprzTransportPutUint8ByAddr(_x) PprzTransportPut1ByteByAddr(_x)
#define PprzTransportPutInt16ByAddr(_x) PprzTransportPut2ByteByAddr(_x)
#define PprzTransportPutUint16ByAddr(_x) PprzTransportPut2ByteByAddr(_x)
#define PprzTransportPutInt32ByAddr(_x) PprzTransportPut4ByteByAddr(_x)
#define PprzTransportPutUint32ByAddr(_x) PprzTransportPut4ByteByAddr(_x)
#define PprzTransportPutFloatByAddr(_x) PprzTransportPut4ByteByAddr(_x)

#define PprzTransportPut(_put, _n, _x) { \
  int i; \
  PprzTransportPutUint8(_n); \
  for(i = 0; i < _n; i++) { \
    _put(&_x[i]); \
  } \
}

#define PprzTransportPutInt16Array(_n, _x) PprzTransportPut(PprzTransportPutInt16ByAddr, _n, _x)

#define PprzTransportPutUint16Array(_n, _x) PprzTransportPut(PprzTransportPutUint16ByAddr, _n, _x)

#endif /* PPRZ_TRANSPORT_H */

