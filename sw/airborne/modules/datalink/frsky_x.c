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

#include "modules/radio_control/cc2500_frsky/cc2500_smartport.h"
#include "modules/datalink/frsky_x.h"

#include <string.h>

struct frsky_x_serial_periph frsky_x_serial;


/* Serial device */
// Based on usb_ser_hw.c
// Generic fifo implementation
static void fifo_init(fifo_t *fifo, uint8_t *buf, size_t size);
static bool fifo_put(fifo_t *fifo, uint8_t c);
static bool fifo_get(fifo_t *fifo, uint8_t *pc);
static int  fifo_avail(fifo_t *fifo);
static int  fifo_free(fifo_t *fifo);

static void fifo_init(fifo_t *fifo, uint8_t *buf, size_t size) {
  fifo->head = 0;
  fifo->tail = 0;
  fifo->buf = buf;
  fifo->size = size;
}

static bool fifo_put(fifo_t *fifo, uint8_t c) {
  int next;

  // check if FIFO has room
  next = (fifo->head + 1) % fifo->size;
  if (next == fifo->tail) {
    // full
    return false;
  }

  fifo->buf[fifo->head] = c;
  fifo->head = next;

  return true;
}

static bool fifo_get(fifo_t *fifo, uint8_t *pc) {
  int next;

  // check if FIFO has data
  if (fifo->head == fifo->tail) {
    return false;
  }

  next = (fifo->tail + 1) % fifo->size;

  *pc = fifo->buf[fifo->tail];
  fifo->tail = next;

  return true;
}

static int fifo_avail(fifo_t *fifo) {
  return (fifo->size + fifo->head - fifo->tail) % fifo->size;
}

static int fifo_free(fifo_t *fifo) {
  return (fifo->size - 1 - fifo_avail(fifo));
}


// Serial device functions
static int frsky_x_serial_check_free_space(void *p, long *fd, uint16_t len);
static void frsky_x_serial_put_byte(void *p, long fd, uint8_t c);
static void frsky_x_serial_put_buffer(void *p, long fd, const uint8_t *data, uint16_t len);
static void frsky_x_serial_send_message(void *p, long fd);
static int frsky_x_serial_char_available(void *p);
static uint8_t frsky_x_serial_get_byte(void *p);

static int frsky_x_serial_check_free_space(void *p, long *fd, uint16_t len) {
  (void) fd;
  return (fifo_free(&((struct frsky_x_serial_periph *)p)->downlink_fifo) >= len ? true : false);
}

static void frsky_x_serial_put_byte(void *p, long fd, uint8_t c) {
  (void) fd;
  fifo_put(&((struct frsky_x_serial_periph *)p)->downlink_fifo, c);
}

static void frsky_x_serial_put_buffer(void *p, long fd, const uint8_t *data, uint16_t len) {
  for (int i = 0; i < len; ++i) {
    frsky_x_serial_put_byte(p, fd, data[i]);
  }
}

static void frsky_x_serial_send_message(void *p, long fd) {
  (void) p;
  (void) fd;
  /* Do nothing, handled by smartPortDownlink_cb */
}

static int frsky_x_serial_char_available(void *p) {
  return fifo_avail(&((struct frsky_x_serial_periph *)p)->uplink_fifo);
}

static uint8_t frsky_x_serial_get_byte(void *p) {
  uint8_t byte;
  fifo_get(&((struct frsky_x_serial_periph *)p)->uplink_fifo, &byte);
  return byte;
}


/* SmartPort sensor */
static bool smartPortDownlink_cb(uint32_t *data) {
  if (fifo_avail(&frsky_x_serial.downlink_fifo) >= 4) {
    uint8_t data_u8[4];
    fifo_get(&frsky_x_serial.downlink_fifo, data_u8 + 0);
    fifo_get(&frsky_x_serial.downlink_fifo, data_u8 + 1);
    fifo_get(&frsky_x_serial.downlink_fifo, data_u8 + 2);
    fifo_get(&frsky_x_serial.downlink_fifo, data_u8 + 3);
    *data =
        (data_u8[0] << 24) |
        (data_u8[1] << 16) |
        (data_u8[2] << 8) |
        (data_u8[3]);
    return true;
  }
  return false;
}


static void smartPortUplink_cb(smartPortPayload_t *payload) {
  uint8_t shift = 24;
  for (uint8_t i = 0; i < payload->frameId; ++i) {
    fifo_put(&frsky_x_serial.uplink_fifo, (uint8_t)((payload->data >> shift) & 0xFF));
    shift -= 8;
  }
}


void datalink_frsky_x_init(void) {
  /* Set up link device */
  fifo_init(&frsky_x_serial.downlink_fifo, frsky_x_serial.downlink_buf, DOWNLINK_BUFFER_SIZE);
  fifo_init(&frsky_x_serial.uplink_fifo, frsky_x_serial.uplink_buf, UPLINK_BUFFER_SIZE);
  frsky_x_serial.device.periph = (void *)(&frsky_x_serial);
  frsky_x_serial.device.check_free_space = frsky_x_serial_check_free_space;
  frsky_x_serial.device.put_byte = frsky_x_serial_put_byte;
  frsky_x_serial.device.put_buffer = frsky_x_serial_put_buffer;
  frsky_x_serial.device.send_message = frsky_x_serial_send_message;
  frsky_x_serial.device.char_available = frsky_x_serial_char_available;
  frsky_x_serial.device.get_byte = frsky_x_serial_get_byte;

  /* Attach to SmartPort hooks */
  smartPortDownlink = smartPortDownlink_cb;
  smartPortUplink = smartPortUplink_cb;
}
