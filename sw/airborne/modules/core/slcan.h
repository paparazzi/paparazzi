/*
 * Copyright (C) 2025 Fabien-B <fabien-b@github.com>
 *
 * This file is part of paparazzi
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
 */

/** @file "modules/core/slcan.h"
 * @author Fabien-B <fabien-b@github.com>
 * SLCAN interface. CAN over serial link. See https://www.canusb.com/files/can232_v3.pdf
Useful for using DroneCAN GUI over serial USB.
 */

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include "mcu_periph/can.h"
#include "core/threads.h"

#define SLCAN_CMD_BUF_LEN (2*64+15)

typedef struct
{
  struct link_device* dev;
  uint8_t cmd_buf[SLCAN_CMD_BUF_LEN]; // large enough for a 64 bytes frame
  uint8_t cmd_buf_idx;
  uint8_t tx_buf[SLCAN_CMD_BUF_LEN]; // large enough for a 64 bytes frame
  bool timestamp;

  struct pprzaddr_can can_if;
  struct pprzcan_frame rx_frame;   // frame received from CAN, to be sent through SLCAN

  pprz_mutex_t mtx;
  bool send_rx;
} slcan_t;


void slcan_init(void);
void slcan_event(void);
