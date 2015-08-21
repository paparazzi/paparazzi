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
  BLUEGIGA_SCANNING               /**< The com is switched from data link to rssi scanning */
};

#define BLUEGIGA_BUFFER_SIZE 256    // buffer max value: 256

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
};

// DEVICE passed to all DOWNLINK_SEND functions
extern struct bluegiga_periph bluegiga_p;

void bluegiga_init(void);
bool_t bluegiga_check_free_space(int len);
void bluegiga_transmit(uint8_t data);
void bluegiga_receive(void);
void bluegiga_send(void);
void bluegiga_increment_buf(uint8_t *buf_idx, uint8_t len);

extern signed char bluegiga_rssi[];    // values initialized with 127
void bluegiga_scan(void);

// Device interface macros
#define BlueGigaCheckFreeSpace() (((bluegiga_p.tx_insert_idx+1)%BLUEGIGA_BUFFER_SIZE) != bluegiga_p.tx_extract_idx)
#define BlueGigaTransmit(_x) bluegiga_transmit(_x)
#define BlueGigaSendMessage() bluegiga_send()
#define BlueGigaChAvailable() (bluegiga_p.rx_extract_idx != bluegiga_p.rx_insert_idx)
#define BlueGigaGetch() bluegiga_getch()

// BLUEGIGA is using pprz_transport
// FIXME it should not appear here, this will be fixed with the rx improvements some day...
// BLUEGIGA needs a specific read_buffer function
#include "subsystems/datalink/pprz_transport.h"
#include "led.h"
static inline void bluegiga_read_buffer(struct pprz_transport *t)
{
  do {
    int c = 0;
    do {
      parse_pprz(t, bluegiga_p.rx_buf[(bluegiga_p.rx_extract_idx + c++) % BLUEGIGA_BUFFER_SIZE]);
    } while (((bluegiga_p.rx_extract_idx + c) % BLUEGIGA_BUFFER_SIZE != bluegiga_p.rx_insert_idx)
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
    bluegiga_increment_buf(&bluegiga_p.rx_extract_idx, c);
  } while (BlueGigaChAvailable()); // continue till all messages read
}

// transmit previous data in buffer and parse data received
#define BlueGigaCheckAndParse(_dev,_trans) {      \
    if (BlueGigaChAvailable())                    \
      bluegiga_read_buffer( &(_trans) );          \
    bluegiga_send();                              \
  }

#endif /* BLUEGIGA_DATA_LINK_H */
