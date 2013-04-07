/*
 * Copyright (C) 2012 Xavier Gibert
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

/** \file mavlink_transport.h
 */

#ifndef MAVLINK_TRANSPORT_H
#define MAVLINK_TRANSPORT_H

#include <inttypes.h>
#include "std.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/transport.h"

/* MAVLINK Transport
 * downlink macros
 */
extern uint8_t mavlink_down_packet_seq, temp_msg_id;
extern uint16_t checksum;

extern uint8_t crc_extra[256];

#define STXMAV  0xFE

/** 4 = STXMAV + len + { packet_seq + AC/ID + COMPONENT/ID + MSG/ID } + ck_a + ck_b */
#define MavlinkTransportSizeOf(_dev, _payload) (_payload-4)

/* MAVLINK CHECKSUM */
#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8
/**
 * @brief Accumulate the X.25 CRC by adding one char at a time.
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new char to hash
 * @param crcAccum the already accumulated checksum
 **/
#define crc_accumulate(_data, _crcAccum)\
{\
  uint8_t tmp;\
  tmp = _data ^ (uint8_t)(_crcAccum &0xff);\
  tmp ^= (tmp<<4);\
  _crcAccum = (_crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);\
}

/**
 * @brief Initiliaze the buffer for the X.25 CRC
 *
 * @param crcAccum the 16 bit X.25 CRC
 */
#define crc_init(_crcAccum)\
{\
	_crcAccum = X25_INIT_CRC;\
}


#define MavlinkTransportCheckFreeSpace(_dev, _x) TransportLink(_dev, CheckFreeSpace(_x))
#define MavlinkTransportPut1Byte(_dev, _x) TransportLink(_dev, Transmit(_x))
#define MavlinkTransportSendMessage(_dev) TransportLink(_dev, SendMessage())

#define MavlinkTransportPutUint8(_dev, _byte) { \
    MavlinkTransportPut1Byte(_dev, _byte);		  \
    crc_accumulate(_byte, checksum); \
 }

#define MavlinkTransportHeader(_dev, payload_len) { \
  MavlinkTransportPut1Byte(_dev, STXMAV);				\
  uint8_t msg_len = MavlinkTransportSizeOf(_dev, payload_len);	\
  crc_init(checksum);			\
  MavlinkTransportPutUint8(_dev, msg_len);			\
}

#define MavlinkTransportPutAcId(_dev, _byte) { \
    MavlinkTransportPutUint8(_dev, _byte);		  \
 }

#define MavlinkTransportPutPacketSequence(_dev) { \
    mavlink_down_packet_seq++;	\
    if(mavlink_down_packet_seq==0){ mavlink_down_packet_seq++; } \
    MavlinkTransportPutUint8(_dev, mavlink_down_packet_seq); \
}

#define MavlinkTransportPutNamedUint8(_dev, _name, _byte) { MavlinkTransportPutUint8(_dev, _byte); temp_msg_id=_byte;}
#define MavlinkTransportPutClassUint8(_dev, _name, _byte) MavlinkTransportPutUint8(_dev, _byte)

#define MavlinkTransportPut1ByteByAddr(_dev, _byte) {	 \
    uint8_t _x = *(_byte);		 \
    MavlinkTransportPutUint8(_dev, _x);	 \
  }

#define MavlinkTransportPut2ByteByAddr(_dev, _byte) { \
    MavlinkTransportPut1ByteByAddr(_dev, _byte);	\
    MavlinkTransportPut1ByteByAddr(_dev, (const uint8_t*)_byte+1);	\
  }

#define MavlinkTransportTrailer(_dev) { \
  uint8_t extra_crc = crc_extra[temp_msg_id] ;\
  crc_accumulate(extra_crc, checksum); \
  uint8_t low = (uint8_t)checksum;\
  uint8_t high = (uint8_t)(checksum>>8);\
  MavlinkTransportPut1Byte(_dev, low);	\
  MavlinkTransportPut1Byte(_dev, high);	\
  MavlinkTransportSendMessage(_dev) \
}

#define MavlinkTransportPut4ByteByAddr(_dev, _byte) { \
    MavlinkTransportPut2ByteByAddr(_dev, _byte);	\
    MavlinkTransportPut2ByteByAddr(_dev, (const uint8_t*)_byte+2);	\
  }

#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */
#define MavlinkTransportPutDoubleByAddr(_dev, _byte) { \
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#define MavlinkTransportPutUint64ByAddr(_dev, _byte) { \
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#define MavlinkTransportPutInt64ByAddr(_dev, _byte) { \
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#else
#define MavlinkTransportPutDoubleByAddr(_dev, _byte) { \
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#define MavlinkTransportPutUint64ByAddr(_dev, _byte) { \
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#define MavlinkTransportPutInt64ByAddr(_dev, _byte) { \
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#endif


#define MavlinkTransportPutInt8ByAddr(_dev, _x) MavlinkTransportPut1ByteByAddr(_dev, _x)
#define MavlinkTransportPutUint8ByAddr(_dev, _x) MavlinkTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)
#define MavlinkTransportPutInt16ByAddr(_dev, _x) MavlinkTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define MavlinkTransportPutUint16ByAddr(_dev, _x) MavlinkTransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define MavlinkTransportPutInt32ByAddr(_dev, _x) MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define MavlinkTransportPutUint32ByAddr(_dev, _x) MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define MavlinkTransportPutFloatByAddr(_dev, _x) MavlinkTransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define MavlinkTransportPutCharByAddr(_dev, _x) MavlinkTransportPut1ByteByAddr(_dev, (const uint8_t*)_x)

#define MavlinkTransportPutArray(_dev, _put, _n, _x) { \
  uint8_t _i; \
  MavlinkTransportPutUint8(_dev, _n); \
  for(_i = 0; _i < _n; _i++) { \
    _put(_dev, &_x[_i]); \
  } \
}

#define MavlinkTransportPutInt8Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutInt8ByAddr, _n, _x)
#define MavlinkTransportPutUint8Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutUint8ByAddr, _n, _x)

#define MavlinkTransportPutCharArray(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutCharByAddr, _n, _x)

#define MavlinkTransportPutInt16Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutInt16ByAddr, _n, _x)
#define MavlinkTransportPutUint16Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutUint16ByAddr, _n, _x)

#define MavlinkTransportPutInt32Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutInt32ByAddr, _n, _x)
#define MavlinkTransportPutUint32Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutUint32ByAddr, _n, _x)

#define MavlinkTransportPutFloatArray(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutFloatByAddr, _n, _x)

#define MavlinkTransportPutInt64Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutInt64ByAddr, _n, _x)
#define MavlinkTransportPutUint64Array(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutUint64ByAddr, _n, _x)

#define MavlinkTransportPutDoubleArray(_dev, _n, _x) MavlinkTransportPutArray(_dev, MavlinkTransportPutDoubleByAddr, _n, _x)




/** Receiving mavlink messages */

// PPRZ parsing state machine
#define MAV_UNINIT      0
#define GOT_STXMAV     1
#define MAV_GOT_LENGTH  2
#define MAV_GOT_PAYLOAD 3
#define MAV_GOT_CRC1    4

struct mavlink_transport {
  // generic interface
  struct transport trans;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};

extern struct mavlink_transport mavlink_tp;

static inline void parse_mavlink(struct mavlink_transport * t, uint8_t c ) {
  switch (t->status) {
    case MAV_UNINIT:
      if (c == STXMAV)
        t->status++;
      break;
    case GOT_STXMAV:
      if (t->trans.msg_received) {
        t->trans.ovrn++;
        goto error;
      }
      t->trans.payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
      t->ck_a = t->ck_b = c;
      t->status++;
      t->payload_idx = 0;
      break;
    case MAV_GOT_LENGTH:
      t->trans.payload[t->payload_idx] = c;
      t->ck_a += c; t->ck_b += t->ck_a;
      t->payload_idx++;
      if (t->payload_idx == t->trans.payload_len)
        t->status++;
      break;
    case MAV_GOT_PAYLOAD:
      if (c != t->ck_a)
        goto error;
      t->status++;
      break;
    case MAV_GOT_CRC1:
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
  t->status = MAV_UNINIT;
  return;
}

static inline void mavlink_parse_payload(struct mavlink_transport * t) {
  uint8_t i;
  for(i = 0; i < t->trans.payload_len; i++)
    dl_buffer[i] = t->trans.payload[i];
  dl_msg_available = TRUE;
  if((dl_buffer[0]!=t->trans.packet_seq+1)&&(dl_buffer[0]!=0)){
    if((t->trans.packet_seq+1)<dl_buffer[0]){
      //uint8_t jump = dl_buffer[0]-(t->trans.packet_seq+1);
      //XGGDEBUG:SEQ: Do something like increment counter
    }else{
      //uint8_t jump = dl_buffer[0]+(255-(t->trans.packet_seq));
      //XGGDEBUG:SEQ: Do something like increment counter
    }
  }
  t->trans.packet_seq = dl_buffer[0];
}


#define MavlinkBuffer(_dev) TransportLink(_dev,ChAvailable())
#define ReadMavlinkBuffer(_dev,_trans) { while (TransportLink(_dev,ChAvailable())&&!(_trans.trans.msg_received)) parse_mavlink(&(_trans),TransportLink(_dev,Getch())); }
#define MavlinkCheckAndParse(_dev,_trans) {  \
  if (MavlinkBuffer(_dev)) {                 \
    ReadMavlinkBuffer(_dev,_trans);          \
    if (_trans.trans.msg_received) {         \
      mavlink_parse_payload(&(_trans));      \
      _trans.trans.msg_received = FALSE;     \
    }                                        \
  }                                          \
}


#endif /* MAVLINK_TRANSPORT_H */

