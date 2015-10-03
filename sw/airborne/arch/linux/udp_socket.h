/*
 * Copyright (C) 2009-2015 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file arch/linux/udp_socket.h
 *
 * Easily create and use UDP sockets.
 * Basically just some convenience functions around a UDP socket.
 */

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <netinet/in.h>
#include "std.h"

struct UdpSocket {
  int sockfd;
  struct sockaddr_in addr_in;
  struct sockaddr_in addr_out;
};

/**
 * Create UDP network (in/out sockets).
 * @param[out] sock   pointer to already allocated UdpSocket struct
 * @param[in]  host      hostname/address
 * @param[in]  port_out  output port
 * @param[in]  port_in   input port (set to < 0 to disable)
 * @param[in]  broadcast if TRUE enable broadcasting
 * @return -1 on error, otherwise 0
 */
extern int udp_socket_create(struct UdpSocket *sock, char *host, int port_out, int port_in, bool_t broadcast);

/**
 * Send a packet from buffer, blocking.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[in] buffer   buffer to send
 * @param[in] len     buffer length in bytes
 * @return number of bytes sent (-1 on error)
 */
extern int udp_socket_send(struct UdpSocket *sock, uint8_t *buffer, uint32_t len);

/**
 * Send a packet from buffer, non-blocking.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[in] buffer   buffer to send
 * @param[in] len      buffer length in bytes
 * @return number of bytes sent (-1 on error)
 */
extern int udp_socket_send_dontwait(struct UdpSocket *sock, uint8_t *buffer, uint32_t len);

/**
 * Receive a UDP packet, dont wait.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[out] buffer  buffer to write received packet to
 * @param[in] len     buffer length in bytes
 * @return number of bytes received (-1 on error)
 */
extern int udp_socket_recv_dontwait(struct UdpSocket *sock, uint8_t *buffer, uint32_t len);

/**
 * Receive one UDP packet.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[out] buffer  buffer to write received packet to
 * @param[in] len     buffer length in bytes
 * @return number of bytes received (-1 on error)
 */
extern int udp_socket_recv(struct UdpSocket *sock, uint8_t *buffer, uint32_t len);

extern int udp_socket_subscribe_multicast(struct UdpSocket *sock, const char* multicast_addr);
extern int udp_socket_set_recvbuf(struct UdpSocket *sock, int buf_size);

#endif /* UDP_SOCKET_H */
