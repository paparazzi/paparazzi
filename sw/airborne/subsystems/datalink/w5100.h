/*
 * Copyright (C) 2012 Gerard Toonstra
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

/**
 * @file subsystems/datalink/w5100.h
 * W5100 ethernet chip I/O
 */

#ifndef W5100_TELEM_H
#define W5100_TELEM_H

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"

#include "subsystems/datalink/transport.h"

#define W5100_RX_BUFFER_SIZE 80
#define W5100_TX_BUFFER_SIZE 80
#define W5100_BUFFER_NUM 2
#define STX 0x99

enum W5100Status {
  W5100StatusUninit,
  W5100StatusIdle,
  W5100StatusReading
};

struct w5100_periph {
  volatile enum W5100Status status;
  int curbuf;
  /* Receive buffer */
  volatile uint8_t rx_buf[W5100_BUFFER_NUM][W5100_RX_BUFFER_SIZE];
  volatile uint16_t rx_insert_idx[W5100_BUFFER_NUM];
  volatile uint16_t rx_extract_idx[W5100_BUFFER_NUM];
  /* Transmit buffer */
  volatile uint8_t tx_buf[W5100_BUFFER_NUM][W5100_TX_BUFFER_SIZE];
  volatile uint16_t tx_insert_idx[W5100_BUFFER_NUM];
  volatile uint16_t tx_extract_idx[W5100_BUFFER_NUM];
  volatile uint8_t work_tx[4];
  volatile uint8_t work_rx[4];
  uint8_t tx_running;
};

extern uint8_t w5100_rx_buf[W5100_RX_BUFFER_SIZE];

extern struct w5100_periph chip0;
//extern uint8_t ck_a, ck_b;

void w5100_init( void );

void w5100_transmit( uint8_t data );
uint16_t w5100_receive( uint8_t *buf, uint16_t len );
void w5100_send( void );
uint16_t w5100_rx_size( uint8_t _s );
bool_t w5100_ch_available( void );

// Defines that are done in mcu_periph on behalf of uart.
// We need to do these here...
#define W5100Init() w5100_init()
#define W5100CheckFreeSpace(_x) (TRUE) // w5100_check_free_space(_x)
#define W5100Transmit(_x) w5100_transmit(_x)
#define W5100SendMessage() w5100_send()
#define W5100ChAvailable() w5100_ch_available()
#define W5100Getch() w5100_getch()
#define W5100TxRunning chip0.tx_running
#define W5100SetBaudrate(_b) w5100_set_baudrate(_b)

#define W5100TransportSizeOf(_dev, _x) (_x+4)
#define W5100TransportCheckFreeSpace(_dev, x) TransportLink(_dev, CheckFreeSpace(x))
#define W5100TransportPut1Byte(_dev, x) TransportLink(_dev, Transmit(x))
#define W5100TransportSendMessage(_dev) TransportLink(_dev, SendMessage())

#define W5100TransportPutUint8(_dev, _byte) {   \
    ck_a += _byte;                              \
    ck_b += ck_a;                               \
    W5100TransportPut1Byte(_dev, _byte);        \
  }

#define W5100TransportPut1ByteByAddr(_dev, _byte) { \
    uint8_t _x = *(_byte);                          \
    W5100TransportPutUint8(_dev, _x);               \
  }

#define W5100TransportPut2Bytes(_dev, _x) {     \
    uint16_t x16 = _x;                          \
    W5100TransportPut1Byte(_dev, x16>>8);       \
    W5100TransportPut1Byte(_dev, x16 & 0xff);   \
  }

#define W5100TransportPut2ByteByAddr(_dev, _byte) {                 \
    W5100TransportPut1ByteByAddr(_dev, _byte);                      \
    W5100TransportPut1ByteByAddr(_dev, (const uint8_t*)_byte+1);    \
  }

#define W5100TransportPut4ByteByAddr(_dev, _byte) {                 \
    W5100TransportPut2ByteByAddr(_dev, _byte);                      \
    W5100TransportPut2ByteByAddr(_dev, (const uint8_t*)_byte+2);    \
  }

#ifdef __IEEE_BIG_ENDIAN // From machine/ieeefp.h
#define W5100TransportPutDoubleByAddr(_dev, _byte) {                \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);    \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);      \
  }
#define W5100TransportPutUint64ByAddr(_dev, _byte) { \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#define W5100TransportPutInt64ByAddr(_dev, _byte) { \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
  }
#else
#define W5100TransportPutDoubleByAddr(_dev, _byte) {                \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);      \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);    \
  }
#define W5100TransportPutUint64ByAddr(_dev, _byte) { \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#define W5100TransportPutInt64ByAddr(_dev, _byte) { \
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte);	\
    W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_byte+4);	\
  }
#endif


#define W5100TransportPutInt8ByAddr(_dev, _x) W5100TransportPut1ByteByAddr(_dev, _x)
#define W5100TransportPutUint8ByAddr(_dev, _x) W5100TransportPut1ByteByAddr(_dev, (const uint8_t*)_x)
#define W5100TransportPutInt16ByAddr(_dev, _x) W5100TransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define W5100TransportPutUint16ByAddr(_dev, _x) W5100TransportPut2ByteByAddr(_dev, (const uint8_t*)_x)
#define W5100TransportPutInt32ByAddr(_dev, _x) W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define W5100TransportPutUint32ByAddr(_dev, _x) W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define W5100TransportPutFloatByAddr(_dev, _x) W5100TransportPut4ByteByAddr(_dev, (const uint8_t*)_x)
#define W5100TransportPutNamedUint8(_dev, _name, _byte) W5100TransportPutUint8(_dev, _byte)

#define W5100TransportPutArray(_dev, _put, _n, _x) {    \
    uint8_t _i;                                         \
    W5100TransportPutUint8(_dev, _n);                   \
    for(_i = 0; _i < _n; _i++) {                        \
      _put(_dev, &_x[_i]);                              \
    }                                                   \
  }

#define W5100TransportPutInt8Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutInt8ByAddr, _n, _x)
#define W5100TransportPutUint8Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutUint8ByAddr, _n, _x)

#define W5100TransportPutCharArray(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutCharByAddr, _n, _x)

#define W5100TransportPutInt16Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutInt16ByAddr, _n, _x)
#define W5100TransportPutUint16Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutUint16ByAddr, _n, _x)

#define W5100TransportPutInt32Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutInt32ByAddr, _n, _x)
#define W5100TransportPutUint32Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutUint32ByAddr, _n, _x)

#define W5100TransportPutFloatArray(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutFloatByAddr, _n, _x)

#define W5100TransportPutInt64Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutInt64ByAddr, _n, _x)
#define W5100TransportPutUint64Array(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutUint64ByAddr, _n, _x)

#define W5100TransportPutDoubleArray(_dev, _n, _x) W5100TransportPutArray(_dev, W5100TransportPutDoubleByAddr, _n, _x)


#define W5100TransportPutFixedArray(_dev, _put, _n, _x) {   \
    uint8_t _i;                                             \
    for(_i = 0; _i < _n; _i++) {                            \
      _put(_dev, &_x[_i]);                                  \
    }                                                       \
  }

#define W5100TransportPutInt8FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutInt8ByAddr, _n, _x)
#define W5100TransportPutUint8FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutUint8ByAddr, _n, _x)

#define W5100TransportPutCharFixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutCharByAddr, _n, _x)

#define W5100TransportPutInt16FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutInt16ByAddr, _n, _x)
#define W5100TransportPutUint16FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutUint16ByAddr, _n, _x)

#define W5100TransportPutInt32FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutInt32ByAddr, _n, _x)
#define W5100TransportPutUint32FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutUint32ByAddr, _n, _x)

#define W5100TransportPutFloatFixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutFloatByAddr, _n, _x)

#define W5100TransportPutInt64FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutInt64ByAddr, _n, _x)
#define W5100TransportPutUint64FixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutUint64ByAddr, _n, _x)

#define W5100TransportPutDoubleFixedArray(_dev, _n, _x) W5100TransportPutFixedArray(_dev, W5100TransportPutDoubleByAddr, _n, _x)


#define W5100TransportHeader(_dev, payload_len) {               \
    W5100TransportPut1Byte(_dev, STX);                          \
    uint8_t msg_len = W5100TransportSizeOf(_dev, payload_len);  \
    W5100TransportPut1Byte(_dev, msg_len);                      \
    ck_a = msg_len; ck_b = msg_len;                             \
  }

#define W5100TransportTrailer(_dev) {           \
    W5100TransportPut1Byte(_dev, ck_a);         \
    W5100TransportPut1Byte(_dev, ck_b);         \
    W5100TransportSendMessage(_dev);            \
  }


/** Receiving pprz messages */

// PPRZ parsing state machine
#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4

struct w5100_transport {
  // generic interface
  struct transport trans;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};

extern struct w5100_transport w5100_tp;

static inline void parse_w5100(struct w5100_transport * t, uint8_t c ) {
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

static inline void w5100_parse_payload(struct w5100_transport * t) {
  uint8_t i;
  for(i = 0; i < t->trans.payload_len; i++)
    dl_buffer[i] = t->trans.payload[i];
  dl_msg_available = TRUE;
}

static inline void w5100_read_buffer( struct w5100_transport *t ) {
  while ( w5100_ch_available() ) {
    w5100_receive( w5100_rx_buf, W5100_RX_BUFFER_SIZE );
    int c = 0;
    do {
      parse_w5100( t, w5100_rx_buf[ c++ ] );
    } while ( ( t->status != UNINIT ) && !(t->trans.msg_received) );
  }
}

#define W5100Buffer(_dev) TransportLink(_dev,ChAvailable())

#define W5100CheckAndParse(_dev,_trans) {       \
    if (W5100Buffer(_dev)) {                    \
      w5100_read_buffer( &(_trans) );           \
      if (_trans.trans.msg_received) {          \
        w5100_parse_payload(&(_trans));         \
        _trans.trans.msg_received = FALSE;      \
      }                                         \
    }                                           \
  }


#endif /* W5100_TELEM_H */

