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

#include "pprzlink/pprzlink_device.h"
#include "subsystems/datalink/datalink.h"

/* The different statuses the communication can be in */
enum BlueGigaStatus {
  BLUEGIGA_UNINIT,                /**< The com isn't initialized */
  BLUEGIGA_IDLE,                  /**< The com is in idle */
  BLUEGIGA_SENDING,               /**< The com is sending */
  BLUEGIGA_SENDING_BROADCAST      /**< The com is switched from data link to rssi scanning */
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
  uint8_t connected;

};

// DEVICE passed to all DOWNLINK_SEND functions
extern struct bluegiga_periph bluegiga_p;
extern signed char bluegiga_rssi[];    // values initialized with 127

bool bluegiga_ch_available(struct bluegiga_periph *p);
void bluegiga_increment_buf(uint8_t *buf_idx, uint8_t len);

void bluegiga_init(struct bluegiga_periph *p);
void bluegiga_scan(struct bluegiga_periph *p);
void bluegiga_broadcast_msg(struct bluegiga_periph *p, char *msg, uint8_t msg_len);

#endif /* BLUEGIGA_DATA_LINK_H */
