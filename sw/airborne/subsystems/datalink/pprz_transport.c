/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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

#include <inttypes.h>
#include "subsystems/datalink/downlink.h"
#ifndef PPRZ_DATALINK_EXPORT
#include "subsystems/datalink/pprz_transport.h"
#else /* PPRZ_DATALINK_EXPORT defined */
#include "pprz_transport.h"
#endif

struct pprz_transport pprz_tp;

static void put_1byte(struct pprz_transport *trans, struct device *dev, const uint8_t byte)
{
  trans->ck_a_tx += byte;
  trans->ck_b_tx += trans->ck_a_tx;
  dev->transmit(dev->periph, byte);
}

static void put_bytes(struct pprz_transport *trans, struct device *dev, uint8_t len, const void *bytes)
{
  const uint8_t *b = (const uint8_t *) bytes;
  int i;
  for (i = 0; i < len; i++) {
    put_1byte(trans, dev, b[i]);
  }
}

static uint8_t size_of(struct pprz_transport *trans __attribute__((unused)), uint8_t len)
{
  // message length: payload + protocol overhead (STX + len + ck_a + ck_b = 4)
  return len + 4;;
}

static void start_message(struct pprz_transport *trans, struct device *dev, uint8_t payload_len)
{
  dev->transmit(dev->periph, STX);
  const uint8_t msg_len = size_of(trans, payload_len);
  dev->transmit(dev->periph, msg_len);
  trans->ck_a_tx = msg_len;
  trans->ck_b_tx = msg_len;
}

static void end_message(struct pprz_transport *trans, struct device *dev)
{
  dev->transmit(dev->periph, trans->ck_a_tx);
  dev->transmit(dev->periph, trans->ck_b_tx);
  dev->send_message(dev);
}

static void overrun(struct pprz_transport *trans __attribute__((unused)), struct device *dev __attribute__((unused)))
{
  downlink_nb_ovrn++;
}

static void count_bytes(struct pprz_transport *trans __attribute__((unused)), struct device *dev __attribute__((unused)), uint8_t bytes)
{
  downlink_nb_bytes += bytes;
}

static int check_available_space(struct pprz_transport *trans __attribute__((unused)), struct device *dev, uint8_t bytes)
{
  return dev->check_free_space(dev, bytes);
}

void pprz_transport_init(void)
{
  pprz_tp.status = UNINIT;
  pprz_tp.trans_rx.msg_received = FALSE;
  pprz_tp.trans_tx.size_of = (size_of_t) size_of;
  pprz_tp.trans_tx.check_available_space = (check_available_space_t) check_available_space;
  pprz_tp.trans_tx.put_bytes = (put_bytes_t) put_bytes;
  pprz_tp.trans_tx.start_message = (start_message_t) start_message;
  pprz_tp.trans_tx.end_message = (end_message_t) end_message;
  pprz_tp.trans_tx.overrun = (overrun_t) overrun;
  pprz_tp.trans_tx.count_bytes = (count_bytes_t) count_bytes;
  pprz_tp.trans_tx.impl = (void *)(&pprz_tp);
}

