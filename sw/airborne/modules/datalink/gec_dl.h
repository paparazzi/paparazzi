/*
 * Copyright (C) 2017 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** \file modules/datalink/gec_dl.h
 *  \brief Datalink using Galois Embedded Crypto
 */

#ifndef GEC_DL_H
#define GEC_DL_H

#include "pprzlink/pprzlink_transport.h"
#include "pprzlink/pprz_transport.h"
#include "modules/datalink/gec/gec.h"
#include "pprz_mutex.h"

#include "mcu_periph/uart.h"
#if USE_USB_SERIAL || USE_USB_SERIAL_DEBUG
#include "mcu_periph/usb_serial.h"
#endif
#if USE_UDP
#include "mcu_periph/udp.h"
#endif

// MSG_ID for key exchange messages.
// Update if IDs change in messages.xml
#define KEY_EXCHANGE_MSG_ID_UAV 239
#define KEY_EXCHANGE_MSG_ID_GCS 159
#define WHITELIST_LEN 20

/**
 * Whitelist for sending and receiving
 * unencrypted messages
 * (mostly for KEY_EXCHANGE messages)
 */
struct gec_whitelist {
  uint8_t whitelist[WHITELIST_LEN];
  uint8_t idx;
  bool init;
};

/**
 * Auxilliary message buffer for storing
 * precomputed key exchange messages
 */
struct message_buffer {
  uint8_t msg[TRANSPORT_PAYLOAD_LEN];
  uint8_t len;
  bool message_ready;
};

struct gec_transport {
  // pprz encapsulation layer
  struct pprz_transport pprz_tp;

  // generic reception interface
  struct transport_rx trans_rx;

  // generic transmission interface
  struct transport_tx trans_tx;
  // buffered tx message
  uint8_t tx_msg[TRANSPORT_PAYLOAD_LEN];
  uint8_t tx_msg_idx;

  // ecnryption primitives
  struct gec_sts_ctx sts;
  struct gec_whitelist whitelist;

  // optional mutex
  PPRZ_MUTEX(mtx_tx);
};

/** PPRZ transport structure */
extern struct gec_transport gec_tp;

/** Init function */
extern void gec_dl_init(void);

/** Datalink Event */
extern void gec_dl_event(void);

void gec_transport_init(struct gec_transport *t);

/** Parsing a frame data and copy the payload to the datalink buffer */
void gec_check_and_parse(struct link_device *dev, struct gec_transport *trans,
                         uint8_t *buf, bool *msg_available);

void gec_process_sts_msg(struct link_device *dev, struct gec_transport *trans, uint8_t *buf);

void respond_sts(struct link_device *dev, struct gec_transport *trans, uint8_t *buf);
void finish_sts(struct link_device *dev, struct gec_transport *trans,
                uint8_t *buf);

void gec_add_to_whitelist(struct gec_whitelist *whitelist, uint8_t id);
bool gec_is_in_the_whitelist(struct gec_whitelist *whitelist, uint8_t id);
bool gec_decrypt_message(uint8_t *buf, volatile uint8_t *payload_len);
bool gec_encrypt_message(uint8_t *buf, uint8_t *payload_len);

void gec_process_msg1(uint8_t *buf);
bool gec_process_msg3(uint8_t *buf);

#endif /* GEC_DL_H */

