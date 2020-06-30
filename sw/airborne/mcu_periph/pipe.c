/*
 * Copyright (C) 2018 Kirk Scheper <kirkscheper@gmail.com>
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

/** \file mcu_periph/pipe.c
 *  \brief arch independent PIPE API
 *
 */

#include "mcu_periph/pipe.h"

#include <string.h>

/* Print the configurations */
#if defined(USE_PIPE0_WRITER) || defined(USE_PIPE0_READER)
struct pipe_periph pipe0;
#endif

#ifdef USE_PIPE0_WRITER
PRINT_CONFIG_VAR(USE_PIPE0_WRITER)
#endif

#ifdef USE_PIPE0_READER
PRINT_CONFIG_VAR(USE_PIPE0_READER)
#endif

#if defined(USE_PIPE1_WRITER) || defined(USE_PIPE1_READER)
struct pipe_periph pipe1;
#endif

#ifdef USE_PIPE1_WRITER
PRINT_CONFIG_VAR(USE_PIPE1_WRITER)
#endif

#ifdef USE_PIPE1_READER
PRINT_CONFIG_VAR(USE_PIPE1_READER)
#endif

#if defined(USE_PIPE2_WRITER) || defined(USE_PIPE2_READER)
struct pipe_periph pipe2;
#endif

#ifdef USE_PIPE2_WRITER
PRINT_CONFIG_VAR(USE_PIPE2_WRITER)
#endif

#ifdef USE_PIPE2_READER
PRINT_CONFIG_VAR(USE_PIPE2_READER)
#endif

/**
 * Initialize the PIPE peripheral
 */
void pipe_periph_init(struct pipe_periph *p, char *read_name, char *write_name)
{
  p->rx_insert_idx = 0;
  p->rx_extract_idx = 0;
  p->tx_insert_idx = 0;
  p->device.periph = (void *)p;
  p->device.check_free_space = (check_free_space_t) pipe_check_free_space;
  p->device.put_byte = (put_byte_t) pipe_put_byte;
  p->device.put_buffer = (put_buffer_t) pipe_put_buffer;
  p->device.send_message = (send_message_t) pipe_send_message;
  p->device.char_available = (char_available_t) pipe_char_available;
  p->device.get_byte = (get_byte_t) pipe_getch;

  pipe_arch_periph_init(p, read_name, write_name);
}

/**
 * Check if there is enough free space in the transmit buffer.
 * @param p   pointer to PIPE peripheral
 * @param len how many bytes of free space to check for
 * @return number of bytes available or 0 if not enough
 */
int WEAK pipe_check_free_space(struct pipe_periph *p, long *fd __attribute__((unused)), uint16_t len)
{
  int available = PIPE_TX_BUFFER_SIZE - p->tx_insert_idx;
  return available >= len ? available : 0;
}

/**
 * Add one data byte to the tx buffer.
 * @param p    pointer to PIPE peripheral
 * @param data byte to add to tx buffer
 */
void WEAK pipe_put_byte(struct pipe_periph *p, long fd __attribute__((unused)), uint8_t data)
{
  if (p->tx_insert_idx >= PIPE_TX_BUFFER_SIZE) {
    return;  // no room
  }

  p->tx_buf[p->tx_insert_idx] = data;
  p->tx_insert_idx++;
}

void WEAK pipe_put_buffer(struct pipe_periph *p, long fd __attribute__((unused)), const uint8_t *data, uint16_t len)
{
  if (p->tx_insert_idx + len >= PIPE_TX_BUFFER_SIZE) {
    return;  // no room
  }

  memcpy(&(p->tx_buf[p->tx_insert_idx]), data, len);
  p->tx_insert_idx += len;
}
