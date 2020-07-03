/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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
 * along with paparazzi; see the file COPYING. If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef DATALINK_FRSKY_X_H
#define DATALINK_FRSKY_X_H

#include "pprzlink/pprzlink_device.h"

#define DOWNLINK_BUFFER_SIZE 512
#define UPLINK_BUFFER_SIZE 512

typedef struct {
  int         head;
  int         tail;
  uint8_t     *buf;
  size_t      size;
} fifo_t;

struct frsky_x_serial_periph {
  /** Generic device interface */
  struct link_device device;
  /* Downlink fifo */
  fifo_t downlink_fifo;
  uint8_t downlink_buf[DOWNLINK_BUFFER_SIZE];
  /* Uplink fifo */
  fifo_t uplink_fifo;
  uint8_t uplink_buf[UPLINK_BUFFER_SIZE];
};
extern struct frsky_x_serial_periph frsky_x_serial;

void datalink_frsky_x_init(void);

#endif // DATALINK_FRSKY_X_H
