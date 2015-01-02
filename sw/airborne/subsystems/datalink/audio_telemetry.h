/*
 * Paparazzi telemetry via audio channel modem functions
 *
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

#ifndef AUDIO_TELEMETRY_H
#define AUDIO_TELEMETRY_H

#include "inttypes.h"

extern uint8_t audio_telemetry_nb_ovrn;

#include "generated/airframe.h"
#include "audio_telemetry_hw.h"


#define TX_BUF_SIZE     255
extern uint8_t           tx_head;
extern volatile uint8_t  tx_tail;
extern uint8_t           tx_buf[ TX_BUF_SIZE ];

extern uint8_t    tx_byte;
extern uint8_t    tx_byte_idx;


#define AudioTelemetrySendMessage() AUDIO_TELEMETRY_CHECK_RUNNING()

#if TX_BUF_SIZE == 256
#define UPDATE_HEAD() {        \
    tx_head++;           \
  }
#else
#define UPDATE_HEAD() {        \
    tx_head++;           \
    if (tx_head >= TX_BUF_SIZE) tx_head = 0; \
  }
#endif

#define AudioTelemetryCheckFreeSpace(_space) (tx_head>=tx_tail? _space < (TX_BUF_SIZE - (tx_head - tx_tail)) : _space < (tx_tail - tx_head))

#define AudioTelemetryPut1Byte(_byte) { \
    tx_buf[tx_head] = _byte;    \
    UPDATE_HEAD();      \
  }

#define AUDIO_TELEMETRY_LOAD_NEXT_BYTE() { \
    tx_byte = tx_buf[tx_tail]; \
    tx_byte_idx = 0; \
    tx_tail++; \
    if( tx_tail >= TX_BUF_SIZE ) \
      tx_tail = 0; \
  }

#define AudioTelemetryTransmit(_x) Audio_TelemetryPut1Byte(_x)

#endif /* AUDIO_TELEMETRY_H */
