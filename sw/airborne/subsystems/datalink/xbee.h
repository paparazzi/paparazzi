/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2014  Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file subsystems/datalink/xbee.h
 * Maxstream XBee serial input and output
 */

#ifndef XBEE_H
#define XBEE_H

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"
#include "subsystems/datalink/transport.h"

#ifdef XBEE868
#include "subsystems/datalink/xbee868.h"
#else /* Not 868 */
#include "subsystems/datalink/xbee24.h"
#endif

#define XBEE_START 0x7e

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
void xbee_init(void);

/** Status of the API packet receiver automata */
#define XBEE_UNINIT         0
#define XBEE_GOT_START      1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD    4

struct xbee_transport {
  // generic reception interface
  struct transport_rx trans_rx;
  // specific xbee transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t cs_rx;
  uint8_t rssi;
  // generic transmission interface
  struct transport_tx trans_tx;
  // specific pprz transport_tx variables
  uint8_t cs_tx;
};

extern struct xbee_transport xbee_tp;

/** Parsing a XBee API frame */
static inline void parse_xbee(struct xbee_transport *t, uint8_t c)
{
  switch (t->status) {
    case XBEE_UNINIT:
      if (c == XBEE_START) {
        t->status++;
      }
      break;
    case XBEE_GOT_START:
      if (t->trans_rx.msg_received) {
        t->trans_rx.ovrn++;
        goto error;
      }
      t->trans_rx.payload_len = c << 8;
      t->status++;
      break;
    case XBEE_GOT_LENGTH_MSB:
      t->trans_rx.payload_len |= c;
      t->status++;
      t->payload_idx = 0;
      t->cs_rx = 0;
      break;
    case XBEE_GOT_LENGTH_LSB:
      t->trans_rx.payload[t->payload_idx] = c;
      t->cs_rx += c;
      t->payload_idx++;
      if (t->payload_idx == t->trans_rx.payload_len) {
        t->status++;
      }
      break;
    case XBEE_GOT_PAYLOAD:
      if (c + t->cs_rx != 0xff) {
        goto error;
      }
      t->trans_rx.msg_received = TRUE;
      goto restart;
      break;
    default:
      goto error;
  }
  return;
error:
  t->trans_rx.error++;
restart:
  t->status = XBEE_UNINIT;
  return;
}

/** Parsing a frame data and copy the payload to the datalink buffer */
static inline void xbee_parse_payload(struct xbee_transport *t)
{
  switch (t->trans_rx.payload[0]) {
    case XBEE_RX_ID:
    case XBEE_TX_ID: /* Useful if A/C is connected to the PC with a cable */
      XbeeGetRSSI(t->trans_rx.payload);
      uint8_t i;
      for (i = XBEE_RFDATA_OFFSET; i < t->trans_rx.payload_len; i++) {
        dl_buffer[i - XBEE_RFDATA_OFFSET] = t->trans_rx.payload[i];
      }
      dl_msg_available = TRUE;
      break;
    default:
      return;
  }
}

#define XBeeCheckAndParse(_dev, _trans) xbee_check_and_parse(&(_dev).device, &(_trans))

static inline void xbee_check_and_parse(struct link_device *dev, struct xbee_transport *trans)
{
  if (dev->char_available(dev->periph)) {
    while (dev->char_available(dev->periph) && !trans->trans_rx.msg_received) {
      parse_xbee(trans, dev->get_byte(dev->periph));
    }
    if (trans->trans_rx.msg_received) {
      xbee_parse_payload(trans);
      trans->trans_rx.msg_received = FALSE;
    }
  }
}

#endif /* XBEE_H */
