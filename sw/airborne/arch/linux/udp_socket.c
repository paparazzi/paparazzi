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
 */

#include "udp_socket.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <errno.h>

//#define TRACE(type,fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(type,fmt,args...)
#define TRACE_ERROR 1

/**
 * Create UDP network (socket).
 * @param[out] network   pointer to already allocated UdpNetwork struct
 * @param[in]  host      hostname/address
 * @param[in]  port_out  output port
 * @param[in]  port_in   input port (set to < 0 to disable)
 * @param[in]  broadcast if TRUE enable broadcasting
 */
void udp_socket_create(struct UdpNetwork *network, char *host, int port_out, int port_in, bool_t broadcast)
{
  if (network == NULL) {
    return;
  }

  // Create the socket with the correct protocl
  network->sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  int one = 1;
  // Enable reusing of address
  setsockopt(network->sockfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  // Enable broadcasting
  if (broadcast) {
    setsockopt(network->sockfd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  }

  // if an input port was specified, bind to it
  if (port_in >= 0) {
    // Create the input address
    network->addr_in.sin_family = PF_INET;
    network->addr_in.sin_port = htons(port_in);
    network->addr_in.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(network->sockfd, (struct sockaddr *)&network->addr_in, sizeof(network->addr_in));
  }

  // set the output/destination address for use in sendto later
  network->addr_out.sin_family = PF_INET;
  network->addr_out.sin_port = htons(port_out);
  network->addr_out.sin_addr.s_addr = inet_addr(host);
}

/**
 * Send a packet from buffer, blocking.
 * @param[in] network  pointer to UdpNetwork struct
 * @param[in] buffer   buffer to send
 * @param[in] len      buffer length in bytes
 * @return number of bytes sent (-1 on error)
 */
int udp_socket_send(struct UdpNetwork *network, uint8_t *buffer, uint16_t len)
{
  if (network == NULL) {
    return -1;
  }

  ssize_t bytes_sent = sendto(network->sockfd, buffer, len, 0,
                       (struct sockaddr *)&network->addr_out, sizeof(network->addr_out));
  if (bytes_sent != len) {
    TRACE(TRACE_ERROR, "error sending to network %d (%d)\n", (int)bytes_sent, strerror(errno));
  }
  return bytes_sent;
}

/**
 * Receive a UDP packet, dont wait.
 * Sets the MSG_DONTWAIT flag, returns 0 if no data is available.
 * @param[in] network  pointer to UdpNetwork struct
 * @param[out] buffer  buffer to write received packet to
 * @param[in] len      buffer length in bytes
 * @return number of bytes received (-1 on error)
 */
int udp_socket_recv_dontwait(struct UdpNetwork *network, uint8_t *buffer, uint16_t len)
{
  socklen_t slen = sizeof(struct sockaddr_in);
  ssize_t bytes_read = recvfrom(network->sockfd, buffer, len, MSG_DONTWAIT,
                                (struct sockaddr *)&network->addr_in, &slen);

  if (bytes_read == -1) {
    // If not data is available, simply return zero bytes read, no error
    if (errno == EWOULDBLOCK) {
      return 0;
    } else {
      TRACE(TRACE_ERROR, "error reading from network error %d \n", errno);
    }
  }

  return bytes_read;
}

/**
 * Receive one UDP packet, blocking.
 * @param[in] network  pointer to UdpNetwork struct
 * @param[out] buffer  buffer to write received packet to
 * @param[in] len      buffer length in bytes (maximum bytes to read)
 * @return number of bytes received (-1 on error)
 */
int udp_socket_recv(struct UdpNetwork *network, uint8_t *buffer, uint16_t len)
{
  socklen_t slen = sizeof(struct sockaddr_in);
  ssize_t bytes_read = recvfrom(network->sockfd, buffer, len, 0,
                                (struct sockaddr *)&network->addr_in, &slen);

  return bytes_read;
}
