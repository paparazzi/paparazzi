/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/linux/mcu_periph/udp_arch.c
 * linux UDP handling
 */

#include "mcu_periph/udp.h"
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>

static inline void udp_create_socket(int* sock, const int protocol, const bool_t reuse_addr, const bool_t broadcast);


/**
 * Initialize the UDP stream
 */
void udp_arch_periph_init(struct udp_periph* p, char* host, int port_out, int port_in, bool_t broadcast)
{
  struct UdpNetwork* network = malloc(sizeof(struct UdpNetwork));

  if (port_out >= 0) {
    // Create the output socket (enable reuse of the address, and broadcast if necessary)
    udp_create_socket(&network->socket_out, 0, TRUE, broadcast);

    // Setup the output address
    network->addr_out.sin_family = PF_INET;
    network->addr_out.sin_port = htons(port_out);
    network->addr_out.sin_addr.s_addr = inet_addr(host);
  }

  if (port_in >= 0) {
    // Creat the input socket (enable reuse of the address, and disable broadcast)
    udp_create_socket(&network->socket_in, 0, TRUE, FALSE);

    // Create the input address
    network->addr_in.sin_family = PF_INET;
    network->addr_in.sin_port = htons(port_in);
    network->addr_in.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(network->socket_in, (struct sockaddr*)&network->addr_in, sizeof(network->addr_in));
  }

  p->network = (void*)network;
}

/**
 * Read bytes from UDP
 */
void udp_receive(struct udp_periph* p)
{
  int16_t i;
  int16_t available = UDP_RX_BUFFER_SIZE - udp_char_available(p);
  uint8_t buf[UDP_RX_BUFFER_SIZE];
  struct UdpNetwork* network = (struct UdpNetwork*) p->network;

  if (available <= 0) {
    return;  // No space
  }

  socklen_t slen = sizeof(struct sockaddr_in);
  ssize_t byte_read = recvfrom(network->socket_in, buf, UDP_RX_BUFFER_SIZE, MSG_DONTWAIT,
                               (struct sockaddr*)&network->addr_in, &slen);

  if (byte_read > 0) {
    for (i = 0; i < byte_read; i++) {
      p->rx_buf[p->rx_insert_idx] = buf[i];
      p->rx_insert_idx = (p->rx_insert_idx + 1) % UDP_RX_BUFFER_SIZE;
    }
  }
}

/**
 * Send a message
 */
void udp_send_message(struct udp_periph* p)
{
  struct UdpNetwork* network = (struct UdpNetwork*) p->network;

  if (p->tx_insert_idx > 0) {
    ssize_t test __attribute__((unused)) = sendto(network->socket_out, p->tx_buf, p->tx_insert_idx, MSG_DONTWAIT,
                          (struct sockaddr*)&network->addr_out, sizeof(network->addr_out));
    p->tx_insert_idx = 0;
  }
}

/**
 * Create a new udp socket
 */
static inline void udp_create_socket(int* sock, const int protocol, const bool_t reuse_addr, const bool_t broadcast)
{
  // Create the socket with the correct protocl
  *sock = socket(PF_INET, SOCK_DGRAM, protocol);
  int one = 1;

  // Enable reusing of addres
  if (reuse_addr) {
    setsockopt(*sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  }

  // Enable broadcasting
  if (broadcast) {
    setsockopt(*sock, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  }
}
