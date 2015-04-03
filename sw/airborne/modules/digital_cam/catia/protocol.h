/*
 * Copyright (C) 2014 OpenUAS
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
 * @file protocol.h
 *
 * |STX|length|... payload=(length-4) bytes ...|Checksum A|Checksum B|
 *
 * where checksum is computed over length and payload:
 * @code
 * mora_ck_a = mora_ck_b = length
 * for each byte b in payload
 *     mora_ck_a += b;
 *     mora_ck_b += mora_ck_a;
 * @endcode
 */

#ifndef MORA_TRANSPORT_H
#define MORA_TRANSPORT_H

#include <inttypes.h>
#include "std.h"

/////////////////////////////////////////////////////////////////////
// MESSAGES

#define MORA_SHOOT              1
#define MORA_SHOOT_MSG_SIZE     (4*10)

// 7 * 4 bytes int32_t
// nr, lat, lon, h, phi, theta, psi

union dc_shot_union {
  struct {
    int32_t nr;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t phi;
    int32_t theta;
    int32_t psi;
    int32_t vground;
    int32_t course;
    int32_t groundalt;
  } data;
  uint8_t bin[MORA_SHOOT_MSG_SIZE];
  int32_t i[10];
};

#define MORA_BUFFER_EMPTY       2

// 0 bytes payload: null

#define MORA_PAYLOAD            3
#define MORA_PAYLOAD_MSG_SIZE   70


// 72 bytes

#define MORA_STATUS             4
#define MORA_STATUS_MSG_SIZE    (4*2)

// 4*2 bytes
union mora_status_union {
  struct mora_status_struct {
    uint16_t cpu;
    uint16_t threads;
    uint16_t shots;
    uint16_t extra;
  } data;
  uint8_t bin[MORA_STATUS_MSG_SIZE];
};

/////////////////////////////////////////////////////////////////////
// SENDING

extern uint8_t mora_ck_a, mora_ck_b;

#define STX  0x99

#define MoraSizeOf(_payload) (_payload+5)

#define MoraPutUint8( _byte) {     \
    mora_ck_a += _byte;              \
    mora_ck_b += mora_ck_a;          \
    CameraLinkTransmit(_byte);     \
  }

#define MoraHeader(msg_id, payload_len) {           \
    CameraLinkTransmit(STX);                        \
    uint8_t msg_len = MoraSizeOf( payload_len);       \
    CameraLinkTransmit(msg_len);                    \
    mora_ck_a = msg_len; mora_ck_b = msg_len;         \
    MoraPutUint8(msg_id);                             \
  }

#define MoraTrailer() {               \
    CameraLinkTransmit(mora_ck_a);    \
    CameraLinkTransmit(mora_ck_b);    \
  }

#define MoraPut1ByteByAddr( _byte) {  \
    uint8_t _x = *(_byte);              \
    MoraPutUint8( _x);                  \
  }

/////////////////////////////////////////////////////////////////////
// PARSING

struct mora_transport {
  // generic interface
  uint8_t payload[256];
  uint8_t error;
  uint8_t msg_id;
  uint8_t msg_received;
  uint8_t payload_len;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};

extern struct mora_transport mora_protocol;

void parse_mora(struct mora_transport *t, uint8_t c);


#endif

