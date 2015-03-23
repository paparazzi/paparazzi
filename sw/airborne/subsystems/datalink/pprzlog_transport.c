/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file subsystems/datalink/pprzlog_transport.c
 *
 * Building and Paparazzi frames with timestamp for data logger.
 *
 * LOG-message: ABCDEFGHxxxxxxxI
 *   A PPRZ_STX (0x99)
 *   B LENGTH (H->H)
 *   C SOURCE (0=uart0, 1=uart1, 2=i2c0, ...)
 *   D TIMESTAMP_LSB (100 microsec raster)
 *   E TIMESTAMP
 *   F TIMESTAMP
 *   G TIMESTAMP_MSB
 *   H PPRZ_DATA
 *     0 SENDER_ID
 *     1 MSG_ID
 *     2 MSG_PAYLOAD
 *     . DATA (messages.xml)
 *   I CHECKSUM (sum[B->H])
 *
 */

#include <inttypes.h>
#include "subsystems/datalink/pprzlog_transport.h"

struct pprzlog_transport pprzlog_tp;

#define STX_LOG  0x99

static void put_1byte(struct pprzlog_transport *trans, struct link_device *dev, const uint8_t byte)
{
  trans->ck += byte;
  dev->put_byte(dev->periph, byte);
}

static void put_bytes(struct pprzlog_transport *trans, struct link_device *dev,
                      enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                      uint8_t len, const void *bytes)
{
  const uint8_t *b = (const uint8_t *) bytes;
  int i;
  for (i = 0; i < len; i++) {
    put_1byte(trans, dev, b[i]);
  }
}

static void put_named_byte(struct pprzlog_transport *trans, struct link_device *dev,
                           enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)))
{
  put_1byte(trans, dev, byte);
}

static uint8_t size_of(struct pprzlog_transport *trans __attribute__((unused)), uint8_t len)
{
  return len;
}

static void start_message(struct pprzlog_transport *trans, struct link_device *dev, uint8_t payload_len)
{
  dev->put_byte(dev->periph, STX_LOG);
  const uint8_t msg_len = size_of(trans, payload_len);
  trans->ck = 0;
  put_1byte(trans, dev, msg_len);
  put_1byte(trans, dev, 0); // TODO use correct source ID
  uint32_t ts = get_sys_time_usec() / 100;
  put_bytes(trans, dev, DL_TYPE_TIMESTAMP, DL_FORMAT_SCALAR, 4, (uint8_t *)(&ts));
}

static void end_message(struct pprzlog_transport *trans, struct link_device *dev)
{
  dev->put_byte(dev->periph, trans->ck);
  dev->send_message(dev->periph);
}

static void overrun(struct pprzlog_transport *trans __attribute__((unused)),
                    struct link_device *dev __attribute__((unused)))
{
}

static void count_bytes(struct pprzlog_transport *trans __attribute__((unused)),
                        struct link_device *dev __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{
}

static int check_available_space(struct pprzlog_transport *trans __attribute__((unused)), struct link_device *dev,
                                 uint8_t bytes)
{
  return dev->check_free_space(dev->periph, bytes);
}

void pprzlog_transport_init(void)
{
  pprzlog_tp.trans_tx.size_of = (size_of_t) size_of;
  pprzlog_tp.trans_tx.check_available_space = (check_available_space_t) check_available_space;
  pprzlog_tp.trans_tx.put_bytes = (put_bytes_t) put_bytes;
  pprzlog_tp.trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  pprzlog_tp.trans_tx.start_message = (start_message_t) start_message;
  pprzlog_tp.trans_tx.end_message = (end_message_t) end_message;
  pprzlog_tp.trans_tx.overrun = (overrun_t) overrun;
  pprzlog_tp.trans_tx.count_bytes = (count_bytes_t) count_bytes;
  pprzlog_tp.trans_tx.impl = (void *)(&pprzlog_tp);
}

