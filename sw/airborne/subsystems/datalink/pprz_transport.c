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
 * @file subsystems/datalink/pprz_transport.c
 *
 * Building and parsing Paparazzi frames.
 *
 * Pprz frame:
 *
 * |STX|length|... payload=(length-4) bytes ...|Checksum A|Checksum B|
 *
 * where checksum is computed over length and payload:
 * @code
 * ck_A = ck_B = length
 * for each byte b in payload
 *     ck_A += b;
 *     ck_b += ck_A;
 * @endcode
 */

#include <inttypes.h>
#include "subsystems/datalink/downlink.h"
#ifndef PPRZ_DATALINK_EXPORT
#include "subsystems/datalink/pprz_transport.h"
#else /* PPRZ_DATALINK_EXPORT defined */
#include "pprz_transport.h"
#endif

struct pprz_transport pprz_tp;

static void put_1byte(struct pprz_transport *trans, struct link_device *dev, const uint8_t byte)
{
  trans->ck_a_tx += byte;
  trans->ck_b_tx += trans->ck_a_tx;
  dev->put_byte(dev->periph, byte);
}

static void put_bytes(struct pprz_transport *trans, struct link_device *dev,
                      enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                      uint8_t len, const void *bytes)
{
  const uint8_t *b = (const uint8_t *) bytes;
  int i;
  for (i = 0; i < len; i++) {
    put_1byte(trans, dev, b[i]);
  }
}

static void put_named_byte(struct pprz_transport *trans, struct link_device *dev,
                           enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)))
{
  put_1byte(trans, dev, byte);
}

static uint8_t size_of(struct pprz_transport *trans __attribute__((unused)), uint8_t len)
{
  // message length: payload + protocol overhead (STX + len + ck_a + ck_b = 4)
  return len + 4;
}

static void start_message(struct pprz_transport *trans, struct link_device *dev, uint8_t payload_len)
{
  dev->put_byte(dev->periph, STX);
  const uint8_t msg_len = size_of(trans, payload_len);
  dev->put_byte(dev->periph, msg_len);
  trans->ck_a_tx = msg_len;
  trans->ck_b_tx = msg_len;
  dev->nb_msgs++;
}

static void end_message(struct pprz_transport *trans, struct link_device *dev)
{
  dev->put_byte(dev->periph, trans->ck_a_tx);
  dev->put_byte(dev->periph, trans->ck_b_tx);
  dev->send_message(dev->periph);
}

static void overrun(struct pprz_transport *trans __attribute__((unused)),
                    struct link_device *dev)
{
  dev->nb_ovrn++;
}

static void count_bytes(struct pprz_transport *trans __attribute__((unused)),
                        struct link_device *dev, uint8_t bytes)
{
  dev->nb_bytes += bytes;
}

static int check_available_space(struct pprz_transport *trans __attribute__((unused)), struct link_device *dev,
                                 uint8_t bytes)
{
  return dev->check_free_space(dev->periph, bytes);
}

void pprz_transport_init(struct pprz_transport *t)
{
  t->status = UNINIT;
  t->trans_rx.msg_received = FALSE;
  t->trans_tx.size_of = (size_of_t) size_of;
  t->trans_tx.check_available_space = (check_available_space_t) check_available_space;
  t->trans_tx.put_bytes = (put_bytes_t) put_bytes;
  t->trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  t->trans_tx.start_message = (start_message_t) start_message;
  t->trans_tx.end_message = (end_message_t) end_message;
  t->trans_tx.overrun = (overrun_t) overrun;
  t->trans_tx.count_bytes = (count_bytes_t) count_bytes;
  t->trans_tx.impl = (void *)(t);
}

