/*
* Copyright (C) 2012-2013 Freek van Tienen and Dino Hensen
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*
*/

/* Wifi ethernet connection over UDP */

#ifndef WIFI_TELEM_H
#define WIFI_TELEM_H

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"

#include "subsystems/datalink/transport.h"

#define STX 0x99

void wifi_init( void );
void wifi_transmit( uint8_t data );
void wifi_send( void );
void wifi_receive( void );

#define WifiInit() wifi_init()
#define WifiCheckFreeSpace(_x) (TRUE)
#define WifiTransmit(_x) wifi_transmit(_x)
#define WifiSendMessage() wifi_send()

#define WifiTransportSizeOf(_dev, _x) (_x+4)
#define WifiTransportCheckFreeSpace(_dev, x) TransportLink(_dev, CheckFreeSpace(x))
#define WifiTransportPut1Byte(_dev, x) TransportLink(_dev, Transmit(x))
#define WifiTransportSendMessage(_dev) TransportLink(_dev, SendMessage())

#define WifiTransportPutUint8(_dev, _byte) {    \
    ck_a += _byte;                              \
    ck_b += ck_a;                               \
    WifiTransportPut1Byte(_dev, _byte);         \
  }

#define WifiTransportPut1ByteByAddr(_dev, _byte) {  \
    uint8_t _x = *(_byte);                          \
    WifiTransportPutUint8(_dev, _x);                \
  }

#define WifiTransportPut2Bytes(_dev, _x) {      \
    uint16_t x16 = _x;                          \
    WifiTransportPut1Byte(_dev, x16>>8);        \
    WifiTransportPut1Byte(_dev, x16 & 0xff);    \
  }

#define WifiTransportPut2ByteByAddr(_dev, _byte) {                 \
    WifiTransportPut1ByteByAddr(_dev, _byte);                      \
    WifiTransportPut1ByteByAddr(_dev, (const uint8_t*)_byte+1);    \
  }

#define WifiTransportPut4ByteByAddr(_dev, _byte) {                 \
    WifiTransportPut2ByteByAddr(_dev, _byte);                      \
    WifiTransportPut2ByteByAddr(_dev, (const uint8_t*)_byte+2);    \
  }

#ifdef __IEEE_BIG_ENDIAN // From machine/ieeefp.h
#define WifiTransportPutDoubleByAddr(_dev, _byte) {                \
    WifiTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);    \
    WifiTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);      \
  }
#else
#define WifiTransportPutDoubleByAddr(_dev, _byte) {                \
    WifiTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);      \
    WifiTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);    \
  }
#endif


#define WifiTransportPutInt8ByAddr(_dev, _x) WifiTransportPut1ByteByAddr(_dev, _x)
#define WifiTransportPutUint8ByAddr(_dev, _x) WifiTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)
#define WifiTransportPutInt16ByAddr(_dev, _x) WifiTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define WifiTransportPutUint16ByAddr(_dev, _x) WifiTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define WifiTransportPutInt32ByAddr(_dev, _x) WifiTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define WifiTransportPutUint32ByAddr(_dev, _x) WifiTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define WifiTransportPutFloatByAddr(_dev, _x) WifiTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define WifiTransportPutNamedUint8(_dev, _name, _byte) WifiTransportPutUint8(_dev, _byte)

#define WifiTransportPutArray(_dev, _put, _n, _x) { \
    uint8_t _i; \
    WifiTransportPutUint8(_dev, _n); \
    for(_i = 0; _i < _n; _i++) { \
      _put(_dev, &_x[_i]); \
    } \
  }

#define WifiTransportPutInt16Array(_dev, _n, _x) WifiTransportPutArray(_dev, WifiTransportPutInt16ByAddr, _n, _x)

#define WifiTransportPutUint16Array(_dev, _n, _x) WifiTransportPutArray(_dev, WifiTransportPutUint16ByAddr, _n, _x)
#define WifiTransportPutUint8Array(_dev, _n, _x) WifiTransportPutArray(_dev, WifiTransportPutUint8ByAddr, _n, _x)
#define WifiTransportPutFloatArray(_dev, _n, _x) WifiTransportPutArray(_dev, WifiTransportPutFloatByAddr, _n, _x)
#define WifiTransportPutDoubleArray(_dev, _n, _x) WifiTransportPutArray(_dev, WifiTransportPutDoubleByAddr, _n, _x)


#define WifiTransportHeader(_dev, payload_len) { \
    WifiTransportPut1Byte(_dev, STX); \
    uint8_t msg_len = WifiTransportSizeOf(_dev, payload_len); \
    WifiTransportPut1Byte(_dev, msg_len); \
    ck_a = msg_len; ck_b = msg_len; \
  }

#define WifiTransportTrailer(_dev) { \
    WifiTransportPut1Byte(_dev, ck_a); \
    WifiTransportPut1Byte(_dev, ck_b); \
    WifiTransportSendMessage(_dev); \
  }

#define WifiCheckAndParse() {       \
    wifi_receive(); \
  }

#endif /* WIFI_TELEM_H */
