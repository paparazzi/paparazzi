/*
 * Copyright (C) 2012 Xavier Gibert
 * Copyright (C) 2013 Gautier Hattenberger
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

/** @file modules/datalink/mavlink_decoder.h
 *  @brief simple decoder of mavlink message
 */

#ifndef MAVLINK_DECODER_H
#define MAVLINK_DECODER_H

#include "std.h"
#include "subsystems/datalink/transport.h"
#include "mcu_periph/uart.h"

/* MAVLINK Transport
 */

#define STXMAV  0xFE

/** Mavlink v1.0 message structure.
 *
 * Mavlink Msg:
 * STXMAV + len + { packet_seq + SYS/ID + COMPONENT/ID + MSG/ID + Payload } + ck_a + ck_b
 *
 * Payload starts after 4 bytes.
 *
 */
struct mavlink_message {
  uint8_t seq;
  uint8_t sys_id;
  uint8_t comp_id;
  uint8_t msg_id;
  uint8_t *payload;
};

#define MAVLINK_PAYLOAD_OFFSET 4

#define MAVLINK_SEQ_IDX 0
#define MAVLINK_SYS_ID_IDX 1
#define MAVLINK_COMP_ID_IDX 2
#define MAVLINK_MSG_ID_IDX 3

/** MAVLINK CHECKSUM */
#define X25_INIT_CRC 0xffff

#ifndef MAVLINK_NO_CRC_EXTRA
// CRC Extra (!!! staticaly calculated !!!)
extern uint8_t mavlink_crc_extra[256];
#endif

/**
 * @brief Accumulate the X.25 CRC by adding one char at a time.
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new char to hash
 * @param crcAccum the already accumulated checksum
 */
static inline void mavlink_crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
  /*Accumulate one byte of data into the CRC*/
  uint8_t tmp;

  tmp = data ^ (uint8_t)(*crcAccum & 0xff);
  tmp ^= (tmp << 4);
  *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

/**
 * @brief Initiliaze the buffer for the X.25 CRC
 *
 * @param crcAccum the 16 bit X.25 CRC
 */
static inline void mavlink_crc_init(uint16_t *crcAccum)
{
  *crcAccum = X25_INIT_CRC;
}

/**
 * @brief Calculates the X.25 checksum on a byte buffer
 *
 * @param  pBuffer buffer containing the byte array to hash
 * @param  length  length of the byte array
 * @return the checksum over the buffer bytes
 **/
static inline uint16_t mavlink_crc_calculate(const uint8_t *pBuffer, uint16_t length)
{
  uint16_t crcTmp;
  mavlink_crc_init(&crcTmp);
  while (length--) {
    mavlink_crc_accumulate(*pBuffer++, &crcTmp);
  }
  return crcTmp;
}

/** Receiving mavlink messages */

// Mavlink parsing state machine
typedef enum {
  MAVLINK_PARSE_STATE_UNINIT = 0,
  MAVLINK_PARSE_STATE_IDLE,
  MAVLINK_PARSE_STATE_GOT_STX,
  MAVLINK_PARSE_STATE_GOT_LENGTH,
  MAVLINK_PARSE_STATE_GOT_PAYLOAD,
  MAVLINK_PARSE_STATE_GOT_CRC1
} mavlink_parse_state; ///< The state machine for the comm parser

/** Structure to submit a callback
 */
struct mavlink_msg_req {
  uint8_t msg_id;                                 ///< Requested message ID
  struct mavlink_message msg;                     ///< Mavlink message
  void (*callback)(struct mavlink_message *msg);  ///< Callback function
  struct mavlink_msg_req *next;
};

/** Mavlink transport protocol
 */
struct mavlink_transport {
  // generic interface
  struct transport_rx trans;
  // specific mavlink transport variables
  mavlink_parse_state status;
  uint8_t payload_idx;
  uint16_t checksum;
  // linked list of callbacks
  struct mavlink_msg_req *req;
};

extern struct mavlink_transport mavlink_tp;

#if MAVLINK_DECODER_DEBUG
/** Send debug frame over PPRZ telemetry.
 *
 * Activated with MAVLINK_DECODER_DEBUG flag
 */
extern void mavlink_send_debug(struct mavlink_transport *t);
#endif

/** Register a callback for a mavlink message
 */
static inline void mavlink_register_msg(struct mavlink_transport *t, struct mavlink_msg_req *req)
{
  // handle linked list of requests
  req->next = t->req;
  t->req = req;
}

/** Mavlink character parser
 */
static inline void parse_mavlink(struct mavlink_transport *t, uint8_t c)
{
  switch (t->status) {
    case MAVLINK_PARSE_STATE_UNINIT:
      t->status = MAVLINK_PARSE_STATE_IDLE; // directly go to idle state (no break)
    case MAVLINK_PARSE_STATE_IDLE:
      if (c == STXMAV) {
        t->status = MAVLINK_PARSE_STATE_GOT_STX;
        mavlink_crc_init(&(t->checksum));
      }
      break;
    case MAVLINK_PARSE_STATE_GOT_STX:
      if (t->trans.msg_received) {
        t->trans.ovrn++;
        goto error;
      }
      t->trans.payload_len = c +
                             MAVLINK_PAYLOAD_OFFSET; /* Not Counting STX, CRC1 and CRC2, adding LENGTH, SEQ, SYSID, COMPID, MSGID  */
      mavlink_crc_accumulate(c, &(t->checksum));
      t->status = MAVLINK_PARSE_STATE_GOT_LENGTH;
      t->payload_idx = 0;
      break;
    case MAVLINK_PARSE_STATE_GOT_LENGTH:
      t->trans.payload[t->payload_idx] = c;
      mavlink_crc_accumulate(c, &(t->checksum));
      t->payload_idx++;
      if (t->payload_idx == t->trans.payload_len) {
        t->status = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
      }
      break;
    case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
#if MAVLINK_DECODER_DEBUG
      mavlink_send_debug(t);
#endif
#ifndef MAVLINK_NO_CRC_EXTRA
      // add extra CRC
      mavlink_crc_accumulate(mavlink_crc_extra[(t->trans.payload[MAVLINK_MSG_ID_IDX])], &(t->checksum));
#endif
      if (c != (t->checksum & 0xFF)) {
        goto error;
      }
      t->status = MAVLINK_PARSE_STATE_GOT_CRC1;
      break;
    case MAVLINK_PARSE_STATE_GOT_CRC1:
      if (c != (t->checksum >> 8)) {
        goto error;
      }
      t->trans.msg_received = TRUE;
      goto restart;
    default:
      goto error;
  }
  return;
error:
  t->trans.error++;
restart:
  t->status = MAVLINK_PARSE_STATE_IDLE;
  return;
}

static inline void mavlink_parse_payload(struct mavlink_transport *t)
{
  uint8_t i;
  struct mavlink_msg_req *el;
  // test the linked list and call callback if needed
  for (el = t->req; el; el = el->next) {
    if (el->msg_id == t->trans.payload[MAVLINK_MSG_ID_IDX]) {
      // build message out of payload
      el->msg.seq     = t->trans.payload[MAVLINK_SEQ_IDX];
      el->msg.sys_id  = t->trans.payload[MAVLINK_SYS_ID_IDX];
      el->msg.comp_id = t->trans.payload[MAVLINK_COMP_ID_IDX];
      el->msg.msg_id  = t->trans.payload[MAVLINK_MSG_ID_IDX];
      // copy buffer
      for (i = 0; i < t->trans.payload_len; i++) {
        el->msg.payload[i] = t->trans.payload[i + MAVLINK_PAYLOAD_OFFSET];
      }
      // callback
      el->callback(&(el->msg));
    }
  }
}

static inline void mavlink_check_and_parse(struct link_device *dev, struct mavlink_transport *trans)
{
  if (dev->char_available(dev->periph)) {
    while (dev->char_available(dev->periph) && !trans->trans.msg_received) {
      parse_mavlink(trans, dev->get_byte(dev->periph));
    }
    if (trans->trans.msg_received) {
      mavlink_parse_payload(trans);
      trans->trans.msg_received = FALSE;
    }
  }
}

/* Datalink Event Macro */
#define MavlinkDatalinkEvent() mavlink_check_and_parse(&(MAVLINK_UART).device, &mavlink_tp)

#endif /* MAVLINK_DECODER_H */

