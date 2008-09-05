/*
 * $Id$
 *  
 * Copyright (C) 2008  Roman Krashanitsa
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

/* Aerocomm serial input and output */

#ifndef AEROCOMM_H
#define AEROCOMM_H

#include "datalink.h"
#include "airframe.h"

#define GROUND_STATION_ADDR 0x100

extern uint8_t aerocomm_cs;
#define AEROCOMM_PAYLOAD_LEN 256
extern uint8_t aerocomm_payload[AEROCOMM_PAYLOAD_LEN];

extern volatile bool_t aerocomm_msg_received;
extern volatile bool_t aerocomm_confirmation_received;
extern volatile bool_t aerocomm_confirmation_status;
extern volatile uint8_t aerocomm_payload_len;
extern uint8_t aerocomm_rssi;
extern uint8_t aerocomm_ovrn, aerocomm_error;

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
#define AEROCOMM_MY_ADDR AC_ID
void aerocomm_init( void );

#define __AerocommLink(dev, _x) dev##_x
#define _AerocommLink(dev, _x)  __AerocommLink(dev, _x)
#define AerocommLink(_x) _AerocommLink(AEROCOMM_UART, _x)

#define AerocommBuffer() AerocommLink(ChAvailable())
#define ReadAerocommBuffer() { while (AerocommLink(ChAvailable())&&!aerocomm_msg_received) parse_aerocomm(AerocommLink(Getch())); }

#define AerocommPrintString(s) AerocommLink(PrintString(s))
#define AerocommPrintHex16(x) AerocommLink(PrintHex16(x))
#define AerocommTransportPut1Byte(x) AerocommLink(Transmit(x))
#define AerocommTransportCheckFreeSpace(x) (AerocommLink(CheckFreeSpace(x)))
#define AerocommTransportSizeOf(_x) (_x+7)
#define AerocommTransportSendMessage() AerocommLink(SendMessage())

#define AerocommTransportPutUint8(_x) { \
  aerocomm_cs += _x; \
  AerocommTransportPut1Byte(_x); \
}

#define AerocommTransportPut1ByteByAddr(_byte) { \
  uint8_t _x = *(_byte);	\
  AerocommTransportPutUint8(_x);	 \
 }

#define AerocommTransportPut2Bytes(_x) { \
  uint16_t x16 = _x; \
  AerocommTransportPut1Byte(x16>>8); \
  AerocommTransportPut1Byte(x16 & 0xff); \
}

#define AerocommTransportPut2ByteByAddr(_byte) { \
    AerocommTransportPut1ByteByAddr(_byte);	\
    AerocommTransportPut1ByteByAddr((const uint8_t*)_byte+1);	\
  }

#define AerocommTransportPut4ByteByAddr(_byte) { \
    AerocommTransportPut2ByteByAddr(_byte);	\
    AerocommTransportPut2ByteByAddr((const uint8_t*)_byte+2);	\
  }

  
#define AerocommTransportPutInt8ByAddr(_x) AerocommTransportPut1ByteByAddr(_x)
#define AerocommTransportPutUint8ByAddr(_x) AerocommTransportPut1ByteByAddr((const uint8_t*)_x)
#define AerocommTransportPutInt16ByAddr(_x) AerocommTransportPut2ByteByAddr((const uint8_t*)_x)
#define AerocommTransportPutUint16ByAddr(_x) AerocommTransportPut2ByteByAddr((const uint8_t*)_x)
#define AerocommTransportPutInt32ByAddr(_x) AerocommTransportPut4ByteByAddr((const uint8_t*)_x)
#define AerocommTransportPutUint32ByAddr(_x) AerocommTransportPut4ByteByAddr((const uint8_t*)_x)
#define AerocommTransportPutFloatByAddr(_x) AerocommTransportPut4ByteByAddr((const uint8_t*)_x)
#define AerocommTransportPutNamedUint8(_name, _byte) AerocommTransportPutUint8(_byte)

#define AerocommTransportPutArray(_put, _n, _x) { \
  uint8_t j; \
  AerocommTransportPutUint8(_n); \
  for(j = 0; j < _n; j++) { \
    _put(&_x[j]); \
  } \
}

#define AerocommTransportPutInt16Array(_n, _x) AerocommTransportPutArray(AerocommTransportPutInt16ByAddr, _n, _x)

#define AerocommTransportPutUint16Array(_n, _x) AerocommTransportPutArray(AerocommTransportPutUint16ByAddr, _n, _x)
#define AerocommTransportPutUint8Array(_n, _x) AerocommTransportPutArray(AerocommTransportPutUint8ByAddr, _n, _x)


/** Constants for the API protocol */
#define AEROCOMM_TX_ID 0x81
#define AEROCOMM_CONF_ID 0x82
#define AEROCOMM_RX_ID 0x83
#define AEROCOMM_RX_ENCH_ID 0x81
#define AEROCOMM_COUNT 0x48
#define AEROCOMM_RETRIES 0x02


#define AerocommTransportPutTXHeader() { \
  AerocommTransportPutUint8(AEROCOMM_TX_ID); \
}

#define AerocommTransportHeader(_len) { \
  aerocomm_cs = 0; \
  AerocommTransportPut1Byte(_len+2); \
}

#define AerocommTransportTrailer() { \
  AerocommTransportPut1Byte(0xFF); \
  AerocommTransportSendMessage(); \
}


/** Status of the API packet receiver automata */
#define AEROCOMM_UNINIT 0
#define AEROCOMM_GOT_START 1
#define AEROCOMM_GOT_LENGTH 2
#define AEROCOMM_GOT_PAYLOAD 3


/** Parsing a Aerocomm API frame */
static inline void parse_aerocomm( uint8_t c ) {
  static uint8_t aerocomm_status = AEROCOMM_UNINIT;
  static uint8_t cs, payload_idx;
  switch (aerocomm_status) {
  case AEROCOMM_UNINIT:
    if (c == AEROCOMM_RX_ID)
    {
    	aerocomm_payload_len = 5;
    	aerocomm_status++;
    	payload_idx = 0;
    	cs=0;
    	aerocomm_payload[payload_idx] = c;
    	cs += c;
    	payload_idx++;
    }
    else if (c == AEROCOMM_RX_ENCH_ID)
    {
    	aerocomm_payload_len = 7;
    	aerocomm_status++;
    	payload_idx = 0;
    	cs=0;
    	aerocomm_payload[payload_idx] = c;
    	cs += c;
    	payload_idx++;
    }
    else if (c == AEROCOMM_CONF_ID)
    {
    	aerocomm_payload_len = 4;
    	aerocomm_status++;
	aerocomm_status++; //payload length==1+header_length
    	payload_idx = 0;
    	cs=0;
    	aerocomm_payload[payload_idx] = c;
    	cs += c;
    	payload_idx++;
    }
    else ; //nothing
    break;
  case AEROCOMM_GOT_START:
	if (aerocomm_msg_received) {
      		aerocomm_ovrn++;
      		goto error;
    	}
	aerocomm_payload[payload_idx] = c;
	aerocomm_payload_len+=c;
    	cs += c;
	aerocomm_status++;
    	payload_idx++;
    	break;
  case AEROCOMM_GOT_LENGTH:
    	aerocomm_payload[payload_idx] = c;
    	cs += c;
    	payload_idx++;
	if (payload_idx == aerocomm_payload_len)
	{
    		aerocomm_msg_received = TRUE;
    		goto restart;
	}
  }
  return;
 error:
  aerocomm_error++;
 restart:
  aerocomm_status = AEROCOMM_UNINIT;
  return;
}

/** Parsing a frame data and copy the payload to the datalink buffer */
static inline void aerocomm_parse_payload(void) {

  switch (aerocomm_payload[0]) {
  case AEROCOMM_RX_ID:
  {
    uint8_t i;
    aerocomm_rssi = 1;
//while (aerocomm_rssi==1) {i++; i--;}
    for(i = 5; i < aerocomm_payload_len; i++) 
      dl_buffer[i-5] = aerocomm_payload[i];
    dl_msg_available = TRUE;
    break;
  }
  case AEROCOMM_RX_ENCH_ID:
  {
    aerocomm_rssi = aerocomm_payload[3];
    uint8_t i;
    for(i = 7; i < aerocomm_payload_len; i++) 
      dl_buffer[i-7] = aerocomm_payload[i];
    dl_msg_available = TRUE;
    break;
  }
  case AEROCOMM_CONF_ID:
  {
    aerocomm_rssi = aerocomm_payload[2];
    aerocomm_confirmation_received=TRUE;
    aerocomm_confirmation_status=aerocomm_payload[3];
    dl_msg_available = FALSE;
  }
  break;
  default:
    return;
  }
}

#endif /* AEROCOMM_H */
