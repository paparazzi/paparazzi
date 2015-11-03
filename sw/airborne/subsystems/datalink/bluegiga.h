/*
 * Copyright (C) 2015  Kirk Scheper <kirkscheper@gmail.com>
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
 * @file subsystems/datalink/bluegiga.h
 * Bluegiga Bluetooth chip I/O
 */

#ifndef BLUEGIGA_DATA_LINK_H
#define BLUEGIGA_DATA_LINK_H

#include "mcu_periph/link_device.h"

/* The different statuses the communication can be in */
enum BlueGigaStatus {
  BLUEGIGA_UNINIT,                /**< The com isn't initialized */
  BLUEGIGA_IDLE,                  /**< The com is in idle */
  BLUEGIGA_SENDING,               /**< The com is sending */
  BLUEGIGA_BROADCASTING           /**< The com is switched from data link to rssi scanning */
};

#ifndef BLUEGIGA_BUFFER_SIZE
#define BLUEGIGA_BUFFER_SIZE 256    // buffer max value: 256
#elif BLUEGIGA_BUFFER_SIZE < 256
#warning "BLUEGIGA_BUFFER_SIZE may be smaller than possible message length, check subsystems/datalink/bluegiga.c:dev_check_free_space for more information"
#elif BLUEGIGA_BUFFER_SIZE > 256
#error "BLUEGIGA_BUFFER_SIZE not made for sizes larger than 256, check subsystems/datalink/bluegiga.c for more information"
#endif

struct bluegiga_periph {
  /* Receive buffer */
  uint8_t rx_buf[BLUEGIGA_BUFFER_SIZE];
  uint8_t rx_insert_idx;
  uint8_t rx_extract_idx;
  /* Transmit buffer */
  uint8_t tx_buf[BLUEGIGA_BUFFER_SIZE];
  uint8_t tx_insert_idx;
  uint8_t tx_extract_idx;
  /* transmit and receive buffers */
  uint8_t work_tx[20];
  uint8_t work_rx[20];
  /** Generic device interface */
  struct link_device device;

  /* some administrative variables */
  uint32_t bytes_recvd_since_last;
  uint8_t end_of_msg;

};

// DEVICE passed to all DOWNLINK_SEND functions
extern struct bluegiga_periph bluegiga_p;
extern signed char bluegiga_rssi[];    // values initialized with 127

bool_t bluegiga_ch_available(struct bluegiga_periph *p);
void bluegiga_increment_buf(uint8_t *buf_idx, uint8_t len);

void bluegiga_init(struct bluegiga_periph *p);
void bluegiga_scan(struct bluegiga_periph *p);
void bluegiga_request_all_rssi(struct bluegiga_periph *p);

// BLUEGIGA is using pprz_transport
// FIXME it should not appear here, this will be fixed with the rx improvements some day...
// BLUEGIGA needs a specific read_buffer function
#include "subsystems/datalink/pprz_transport.h"
#include "led.h"
static inline void bluegiga_read_buffer(struct bluegiga_periph *p, struct pprz_transport *t)
{
  do {
    uint8_t c = 0;
    do {
      parse_pprz(t, p->rx_buf[(p->rx_extract_idx + c++) % BLUEGIGA_BUFFER_SIZE]);
    } while (((p->rx_extract_idx + c) % BLUEGIGA_BUFFER_SIZE != p->rx_insert_idx)
             && !(t->trans_rx.msg_received));
    // reached end of circular read buffer or message received
    // if received, decode and advance
    if (t->trans_rx.msg_received) {
#ifdef MODEM_LED
      LED_TOGGLE(MODEM_LED);
#endif
      pprz_parse_payload(t);
      t->trans_rx.msg_received = FALSE;
    }
    bluegiga_increment_buf(&p->rx_extract_idx, c);
  } while (bluegiga_ch_available(p)); // continue till all messages read
}

// transmit previous data in buffer and parse data received
#define BlueGigaCheckAndParse(_dev,_trans) {     \
    if (bluegiga_ch_available(&(_dev)))          \
      bluegiga_read_buffer(&(_dev), &(_trans));  \
  }

#endif /* BLUEGIGA_DATA_LINK_H */
