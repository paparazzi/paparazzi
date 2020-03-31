/*
 * Copyright (C) 2014 Freek van tienen <freek.v.tienen@gmail.com>
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

/** \file mcu_periph/udp.c
 *  \brief arch independent UDP API
 *
 */

#include "mcu_periph/udp.h"

#include <string.h>

/* Print the configurations */
#if USE_UDP0
struct udp_periph udp0;
PRINT_CONFIG_VAR(UDP0_HOST)
PRINT_CONFIG_VAR(UDP0_PORT_OUT)
PRINT_CONFIG_VAR(UDP0_PORT_IN)
PRINT_CONFIG_VAR(UDP0_BROADCAST)
#endif // USE_UDP0

#if USE_UDP1
struct udp_periph udp1;
PRINT_CONFIG_VAR(UDP1_HOST)
PRINT_CONFIG_VAR(UDP1_PORT_OUT)
PRINT_CONFIG_VAR(UDP1_PORT_IN)
PRINT_CONFIG_VAR(UDP1_BROADCAST)
#endif // USE_UDP1

#if USE_UDP2
struct udp_periph udp2;
PRINT_CONFIG_VAR(UDP2_HOST)
PRINT_CONFIG_VAR(UDP2_PORT_OUT)
PRINT_CONFIG_VAR(UDP2_PORT_IN)
PRINT_CONFIG_VAR(UDP2_BROADCAST)
#endif // USE_UDP2

/**
 * Initialize the UDP peripheral
 */
void udp_periph_init(struct udp_periph *p, char *host, int port_out, int port_in, bool broadcast)
{
  p->rx_insert_idx = 0;
  p->rx_extract_idx = 0;
  p->tx_insert_idx = 0;
  p->device.periph = (void *)p;
  p->device.check_free_space = (check_free_space_t) udp_check_free_space;
  p->device.put_byte = (put_byte_t) udp_put_byte;
  p->device.put_buffer = (put_buffer_t) udp_put_buffer;
  p->device.send_message = (send_message_t) udp_send_message;
  p->device.char_available = (char_available_t) udp_char_available;
  p->device.get_byte = (get_byte_t) udp_getch;

  // Arch dependent initialization
  udp_arch_periph_init(p, host, port_out, port_in, broadcast);
}

/**
 * Check if there is enough free space in the transmit buffer.
 * @param p   pointer to UDP peripheral
 * @param len how many bytes of free space to check for
 * @return number of bytes available or 0 if not enough
 */
int WEAK udp_check_free_space(struct udp_periph *p, long *fd __attribute__((unused)), uint16_t len)
{
  int available = UDP_TX_BUFFER_SIZE - p->tx_insert_idx;
  return available >= len ? available : 0;
}

/**
 * Add one data byte to the tx buffer.
 * @param p    pointer to UDP peripheral
 * @param data byte to add to tx buffer
 */
void WEAK udp_put_byte(struct udp_periph *p, long fd __attribute__((unused)), uint8_t data)
{
  if (p->tx_insert_idx >= UDP_TX_BUFFER_SIZE) {
    return;  // no room
  }

  p->tx_buf[p->tx_insert_idx] = data;
  p->tx_insert_idx++;
}

void WEAK udp_put_buffer(struct udp_periph *p, long fd __attribute__((unused)), const uint8_t *data, uint16_t len)
{
  if (p->tx_insert_idx + len >= UDP_TX_BUFFER_SIZE) {
    return;  // no room
  }

  memcpy(&(p->tx_buf[p->tx_insert_idx]), data, len);
  p->tx_insert_idx += len;
}
