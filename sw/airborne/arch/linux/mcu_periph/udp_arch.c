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
#include "udp_socket.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

/**
 * Initialize the UDP peripheral.
 * Allocate UdpSocket struct and create and bind the UDP socket.
 */
void udp_arch_periph_init(struct udp_periph *p, char *host, int port_out, int port_in, bool_t broadcast)
{
  struct UdpSocket *sock = malloc(sizeof(struct UdpSocket));
  udp_socket_create(sock, host, port_out, port_in, broadcast);
  p->network = (void *)sock;
}

/**
 * Read bytes from UDP
 */
void udp_receive(struct udp_periph *p)
{
  if (p == NULL) return;
  if (p->network == NULL) return;

  int16_t i;
  int16_t available = UDP_RX_BUFFER_SIZE - udp_char_available(p);
  uint8_t buf[UDP_RX_BUFFER_SIZE];
  struct UdpSocket *sock = (struct UdpSocket *) p->network;

  if (available <= 0) {
    return;  // No space
  }

  socklen_t slen = sizeof(struct sockaddr_in);
  ssize_t byte_read = recvfrom(sock->sockfd, buf, UDP_RX_BUFFER_SIZE, MSG_DONTWAIT,
                               (struct sockaddr *)&sock->addr_in, &slen);

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
void udp_send_message(struct udp_periph *p)
{
  if (p == NULL) return;
  if (p->network == NULL) return;

  struct UdpSocket *sock = (struct UdpSocket *) p->network;

  if (p->tx_insert_idx > 0) {
    ssize_t bytes_sent = sendto(sock->sockfd, p->tx_buf, p->tx_insert_idx, MSG_DONTWAIT,
                                (struct sockaddr *)&sock->addr_out, sizeof(sock->addr_out));
    if (bytes_sent != p->tx_insert_idx) {
      if (bytes_sent < 0) {
        perror("udp_send_message failed");
      }
      else {
        fprintf(stderr, "udp_send_message: only sent %d bytes instead of %d\n",
                (int)bytes_sent, p->tx_insert_idx);
      }
    }
    p->tx_insert_idx = 0;
  }
}

/**
 * Send a packet from another buffer
 */
void udp_send_raw(struct udp_periph *p, uint8_t *buffer, uint16_t size)
{
  if (p == NULL) return;
  if (p->network == NULL) return;

  struct UdpSocket *sock = (struct UdpSocket *) p->network;
  ssize_t test __attribute__((unused)) = sendto(sock->sockfd, buffer, size, MSG_DONTWAIT,
                                         (struct sockaddr *)&sock->addr_out, sizeof(sock->addr_out));
}
