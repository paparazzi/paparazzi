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

/** \file modules/datalink/spprz_dl.c
 *  \brief Datalink using Galois Embedded Crypto
 */

#include "gec_dl.h"
#include "subsystems/datalink/datalink.h"
#include <string.h> // for memset()

struct gec_transport gec_tp;


static struct gec_transport * get_trans(struct pprzlink_msg *msg)
{
  return (struct gec_transport *)(msg->trans->impl);
}

static inline void insert_byte(struct gec_transport *t, const uint8_t byte) {
  t->tx_msg[t->tx_msg_idx] = byte;
  t->tx_msg_idx++;
}


static void put_bytes(struct pprzlink_msg *msg,
                      long fd __attribute__((unused)),
                      enum TransportDataType type __attribute__((unused)),
                      enum TransportDataFormat format __attribute__((unused)),
                      const void *bytes, uint16_t len)
{
  const uint8_t *b = (const uint8_t *) bytes;
  int i;
  for (i = 0; i < len; i++) {
    insert_byte(get_trans(msg), b[i]);
  }
}


static void put_named_byte(struct pprzlink_msg *msg,
                           long fd __attribute__((unused)),
                           enum TransportDataType type __attribute__((unused)),
                           enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)))
{
  insert_byte(get_trans(msg), byte);
}


/**
 * TODO
 */
static uint8_t size_of(struct pprzlink_msg *msg, uint8_t len)
{
  // message length: payload + crypto overhead
  return len + get_trans(msg)->pprz_tp.trans_tx.size_of(msg, len);
  // + ???; TODO depends on crypto header size
}


/**
 * TODO
 */
static void start_message(struct pprzlink_msg *msg,
                          long fd __attribute__((unused)), uint8_t payload_len)
{
  PPRZ_MUTEX_LOCK(get_trans(msg)->mtx_tx); // lock mutex
  memset(get_trans(msg)->tx_msg, 0, TRANSPORT_PAYLOAD_LEN);
  get_trans(msg)->tx_msg_idx = 0;
  // TODO add crypto header to buffer if needed
}

/**
 * TODO
 */
static void end_message(struct pprzlink_msg *msg, long fd)
{
  // TODO apply crypto on buffer here, or a part of it

  // if valid
    // encapsulated encrypted data with pprz
    get_trans(msg)->pprz_tp.trans_tx.start_message(msg, fd, get_trans(msg)->tx_msg_idx);
    for (int i = 0; i < get_trans(msg)->tx_msg_idx; i++) {
      // add byte one by one for now
      get_trans(msg)->pprz_tp.trans_tx.put_bytes(msg, 0, DL_TYPE_UINT8, DL_FORMAT_SCALAR,
          get_trans(msg)->tx_msg[i], 1);
    }
    get_trans(msg)->pprz_tp.trans_tx.end_message(msg, fd);

  // unlock mutex
  PPRZ_MUTEX_UNLOCK(get_trans(msg)->mtx_tx);
}


/**
 * TODO
 */
static void overrun(struct pprzlink_msg *msg)
{
  get_trans(msg)->pprz_tp.trans_tx.overrun(msg);
}

static void count_bytes(struct pprzlink_msg *msg, uint8_t bytes)
{
  get_trans(msg)->pprz_tp.trans_tx.count_bytes(msg, bytes);
}

static int check_available_space(struct pprzlink_msg *msg, long *fd, uint16_t bytes)
{
  return get_trans(msg)->pprz_tp.trans_tx.check_available_space(msg, fd, bytes);
}


// Init pprz transport structure
static void gec_transport_init(struct gec_transport *t)
{
  t->trans_rx.msg_received = false;
  t->trans_tx.size_of = (size_of_t) size_of;
  t->trans_tx.check_available_space = (check_available_space_t) check_available_space;
  t->trans_tx.put_bytes = (put_bytes_t) put_bytes;
  t->trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  t->trans_tx.start_message = (start_message_t) start_message;
  t->trans_tx.end_message = (end_message_t) end_message;
  t->trans_tx.overrun = (overrun_t) overrun;
  t->trans_tx.count_bytes = (count_bytes_t) count_bytes;
  t->trans_tx.impl = (void *)(t);
  PPRZ_MUTEX_INIT(t->mtx_tx); // init mutex, check if correct pointer
}








void gec_dl_init(void)
{
  // init pprz transport
  pprz_transport_init(&gec_tp.pprz_tp);

  // init crypto transport
  gec_transport_init(&gec_tp);
}


void gec_dl_event(void)
{
  pprz_check_and_parse(&DOWNLINK_DEVICE.device, &gec_tp.pprz_tp, gec_tp.trans_rx.payload,
                       &gec_tp.trans_rx.msg_received);
  if (gec_tp.trans_rx.msg_received) {
    // decrypt msg here

    // process decrypted message if needed

    // store in dl_buffer (see macro in datalink.h)
    DatalinkFillDlBuffer(gec_tp.trans_rx.payload, TRANSPORT_PAYLOAD_LEN);

    // pass to datalink
    DlCheckAndParse(&DOWNLINK_DEVICE.device, &gec_tp.trans_tx, dl_buffer, &dl_msg_available);

    // reset flag
    gec_tp.trans_rx.msg_received = false;
  }
}

