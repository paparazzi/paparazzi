/*
 * Copyright (C) 2012 Gerard Toonstra
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file subsystems/datalink/w5100.h
 * W5100 ethernet chip I/O
 */

#ifndef W5100_H
#define W5100_H

#include "pprzlink/pprzlink_device.h"
#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"

#define W5100_RX_BUFFER_SIZE 80
#define W5100_TX_BUFFER_SIZE 80
#define W5100_BUFFER_NUM 2

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
  volatile uint8_t tx_running;
  /** Generic device interface */
  struct link_device device;
};

extern uint8_t w5100_rx_buf[W5100_RX_BUFFER_SIZE];

extern struct w5100_periph chip0;

void w5100_init(void);
void w5100_transmit(uint8_t data);
void w5100_transmit_buffer(uint8_t *data, uint16_t len);
uint16_t w5100_receive(uint8_t *buf, uint16_t len);
void w5100_send(void);
uint16_t w5100_rx_size(uint8_t _s);
bool w5100_ch_available(void);


// W5100 is using pprz_transport
// FIXME it should not appear here, this will be fixed with the rx improvements some day...
// W5100 needs a specific read_buffer function
#include "pprzlink/pprz_transport.h"

static inline void w5100_read_buffer(struct pprz_transport *t)
{
  while (w5100_ch_available()) {
    w5100_receive(w5100_rx_buf, W5100_RX_BUFFER_SIZE);
    int c = 0;
    do {
      parse_pprz(t, w5100_rx_buf[ c++ ]);
    } while ((t->status != UNINIT) && !(t->trans_rx.msg_received));
  }
}

#define W5100CheckAndParse(_dev, _trans) w5100_check_and_parse(&(_dev).device, &(_trans))

static inline void w5100_check_and_parse(struct link_device *dev, struct pprz_transport *trans)
{
  if (dev->char_available(dev->periph)) {
    w5100_read_buffer(trans);
    if (trans->trans_rx.msg_received) {
      DatalinkFillDlBuffer(trans->trans_rx.payload, trans->trans_rx.payload_len);
      trans->trans_rx.msg_received = false;
    }
  }
}

#endif /* W5100_H */

