/*
 * Copyright (C) 2012 The Paparazzi Team
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
 */

/**
 * @file arch/sim/mcu_periph/uart_arch.h
 * Dummy header for handling of UART hardware in sim.
 */

#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <sys/select.h>
#include <unistd.h>
#include <assert.h>

#define STDINOUT_BUFFER_SIZE 256
#define FD_STDIN 0

extern char stdinout_buffer[STDINOUT_BUFFER_SIZE];
extern uint8_t stdinout_rx_insert_idx;
extern uint8_t stdinout_rx_extract_idx;

#define UART_SPEED(_def) {}

static inline bool StdInOutChAvailable(void)
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(FD_STDIN, &fds);
  select(1, &fds, NULL, NULL, &tv);
  if (FD_ISSET(FD_STDIN, &fds)) {
    char tmp_buf[STDINOUT_BUFFER_SIZE];
    uint8_t n = read(FD_STDIN, tmp_buf, STDINOUT_BUFFER_SIZE);
    unsigned int i;
    for (i = 0; i < n; i++) {
      stdinout_buffer[stdinout_rx_insert_idx] = tmp_buf[i];
      stdinout_rx_insert_idx++; /* Auto overflow */
    }
  }
  return (stdinout_rx_insert_idx != stdinout_rx_extract_idx);
}

#define StdInOutTransmit(_char) putchar(_char)
#define StdInOutGetch() ({ \
    assert(stdinout_rx_insert_idx != stdinout_rx_extract_idx); \
    stdinout_buffer[stdinout_rx_extract_idx++]; \
  })
