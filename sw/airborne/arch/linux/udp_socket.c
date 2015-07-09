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
 * @file arch/linux/udp_socket.c
 *
 * Easily create and use UDP sockets.
 */

#include "udp_socket.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

//#define TRACE(type,fmt,args...)    fprintf(stderr, fmt, args)
#define TRACE(type,fmt,args...)
#define TRACE_ERROR 1

/**
 * Create UDP socket and bind it.
 * @param[out] sock   pointer to already allocated UdpSocket struct
 * @param[in]  host      ip address or hostname (hostname not possible if static linking)
 * @param[in]  port_out  output port
 * @param[in]  port_in   input port (set to < 0 to disable)
 * @param[in]  broadcast if TRUE enable broadcasting
 * @return -1 on error, otherwise 0
 */
int udp_socket_create(struct UdpSocket *sock, char *host, int port_out, int port_in, bool_t broadcast)
{
  if (sock == NULL) {
    return -1;
  }

#ifndef LINUX_LINK_STATIC
  /* try to convert host ipv4 address to binary format */
  struct in_addr host_ip;
  if (host[0] != '\0' && !inet_aton(host, &host_ip)) {
    /* not an IP address, try to resolve hostname */
    struct hostent *hp;
    hp = gethostbyname(host);
    if (!hp) {
      fprintf(stderr, "could not obtain address of %s\n", host);
      return -1;
    }
    /* check if IPv4 address */
    if (hp->h_addrtype == AF_INET && hp->h_length == 4) {
      /* simply use first address */
      memcpy(&host_ip.s_addr, hp->h_addr_list[0], hp->h_length);
    } else {
      return -1;
    }
  }
#endif

  // Create the socket with the correct protocl
  sock->sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  int one = 1;
  // Enable reusing of address
  setsockopt(sock->sockfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  // Enable broadcasting
  if (broadcast) {
    setsockopt(sock->sockfd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  }

  // if an input port was specified, bind to it
  if (port_in >= 0) {
    // Create the input address
    sock->addr_in.sin_family = PF_INET;
    sock->addr_in.sin_port = htons(port_in);
    sock->addr_in.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(sock->sockfd, (struct sockaddr *)&sock->addr_in, sizeof(sock->addr_in));
  }

  // set the output/destination address for use in sendto later
  sock->addr_out.sin_family = PF_INET;
  sock->addr_out.sin_port = htons(port_out);
#ifndef LINUX_LINK_STATIC
  sock->addr_out.sin_addr.s_addr = host_ip.s_addr;
#else
  sock->addr_out.sin_addr.s_addr = inet_addr(host);
#endif
  return 0;
}

/**
 * Send a packet from buffer, blocking.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[in] buffer   buffer to send
 * @param[in] len      buffer length in bytes
 * @return number of bytes sent (-1 on error)
 */
int udp_socket_send(struct UdpSocket *sock, uint8_t *buffer, uint16_t len)
{
  if (sock == NULL) {
    return -1;
  }

  ssize_t bytes_sent = sendto(sock->sockfd, buffer, len, 0,
                              (struct sockaddr *)&sock->addr_out, sizeof(sock->addr_out));
  if (bytes_sent != len) {
    TRACE(TRACE_ERROR, "error sending to sock %d (%d)\n", (int)bytes_sent, strerror(errno));
  }
  return bytes_sent;
}

/**
 * Send a packet from buffer, non-blocking.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[in] buffer   buffer to send
 * @param[in] len      buffer length in bytes
 * @return number of bytes sent (-1 on error)
 */
int udp_socket_send_dontwait(struct UdpSocket *sock, uint8_t *buffer, uint16_t len)
{
  if (sock == NULL) {
    return -1;
  }

  ssize_t bytes_sent = sendto(sock->sockfd, buffer, len, MSG_DONTWAIT,
                       (struct sockaddr *)&sock->addr_out, sizeof(sock->addr_out));
  return bytes_sent;
}

/**
 * Receive a UDP packet, dont wait.
 * Sets the MSG_DONTWAIT flag, returns 0 if no data is available.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[out] buffer  buffer to write received packet to
 * @param[in] len      buffer length in bytes
 * @return number of bytes received (-1 on error)
 */
int udp_socket_recv_dontwait(struct UdpSocket *sock, uint8_t *buffer, uint16_t len)
{
  socklen_t slen = sizeof(struct sockaddr_in);
  ssize_t bytes_read = recvfrom(sock->sockfd, buffer, len, MSG_DONTWAIT,
                                (struct sockaddr *)&sock->addr_in, &slen);

  if (bytes_read == -1) {
    // If not data is available, simply return zero bytes read, no error
    if (errno == EWOULDBLOCK) {
      return 0;
    } else {
      TRACE(TRACE_ERROR, "error reading from sock error %d \n", errno);
    }
  }

  return bytes_read;
}

/**
 * Receive one UDP packet, blocking.
 * @param[in] sock  pointer to UdpSocket struct
 * @param[out] buffer  buffer to write received packet to
 * @param[in] len      buffer length in bytes (maximum bytes to read)
 * @return number of bytes received (-1 on error)
 */
int udp_socket_recv(struct UdpSocket *sock, uint8_t *buffer, uint16_t len)
{
  socklen_t slen = sizeof(struct sockaddr_in);
  ssize_t bytes_read = recvfrom(sock->sockfd, buffer, len, 0,
                                (struct sockaddr *)&sock->addr_in, &slen);

  return (int)bytes_read;
}

int udp_socket_subscribe_multicast(struct UdpSocket *sock, const char* multicast_addr) {
  // Create the request
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(multicast_addr);
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);

  // Send the request
  return setsockopt(sock->sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq));
}

int udp_socket_set_recvbuf(struct UdpSocket *sock, int buf_size) {
  // Set and check
  unsigned int optval_size = 4;
  int buf_ret;
  setsockopt(sock->sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&buf_size, optval_size);
  getsockopt(sock->sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&buf_ret, &optval_size);

  if(buf_size != buf_ret)
    return -1;

  return 0;
}
