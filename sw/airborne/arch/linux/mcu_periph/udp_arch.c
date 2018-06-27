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
#include <pthread.h>
#include <sys/select.h>

#include "rt_priority.h"

#ifndef UDP_THREAD_PRIO
#define UDP_THREAD_PRIO 10
#endif

static void *udp_thread(void *data __attribute__((unused)));
static pthread_mutex_t udp_mutex = PTHREAD_MUTEX_INITIALIZER;

void udp_arch_init(void)
{
  pthread_mutex_init(&udp_mutex, NULL);

#ifdef USE_UDP0
  UDP0Init();
#endif
#ifdef USE_UDP1
  UDP1Init();
#endif
#ifdef USE_UDP2
  UDP2Init();
#endif

  pthread_t tid;
  if (pthread_create(&tid, NULL, udp_thread, NULL) != 0) {
    fprintf(stderr, "udp_arch_init: Could not create UDP reading thread.\n");
    return;
  }
#ifndef __APPLE__
  pthread_setname_np(tid, "udp");
#endif
}

/**
 * Initialize the UDP peripheral.
 * Allocate UdpSocket struct and create and bind the UDP socket.
 */
void udp_arch_periph_init(struct udp_periph *p, char *host, int port_out, int port_in, bool broadcast)
{
  struct UdpSocket *sock = malloc(sizeof(struct UdpSocket));
  udp_socket_create(sock, host, port_out, port_in, broadcast);
  p->network = (void *)sock;
}

/**
 * Get number of bytes available in receive buffer.
 * @param p pointer to UDP peripheral
 * @return number of bytes available in receive buffer
 */
uint16_t udp_char_available(struct udp_periph *p)
{
  pthread_mutex_lock(&udp_mutex);
  int16_t available = p->rx_insert_idx - p->rx_extract_idx;
  if (available < 0) {
    available += UDP_RX_BUFFER_SIZE;
  }
  pthread_mutex_unlock(&udp_mutex);
  return (uint16_t)available;
}

/**
 * Get the last character from the receive buffer.
 * @param p pointer to UDP peripheral
 * @return last byte
 */
uint8_t udp_getch(struct udp_periph *p)
{
  pthread_mutex_lock(&udp_mutex);
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UDP_RX_BUFFER_SIZE;
  pthread_mutex_unlock(&udp_mutex);
  return ret;
}

/**
 * Read bytes from UDP
 */
void udp_receive(struct udp_periph *p)
{
  if (p == NULL) { return; }
  if (p->network == NULL) { return; }

  int16_t i;
  int16_t available = UDP_RX_BUFFER_SIZE - udp_char_available(p);
  uint8_t buf[UDP_RX_BUFFER_SIZE];
  struct UdpSocket *sock = (struct UdpSocket *) p->network;

  if (available <= 0) {
    return;  // No space
  }

  socklen_t slen = sizeof(struct sockaddr_in);
  ssize_t byte_read = recvfrom(sock->sockfd, buf, available, MSG_DONTWAIT,
                               (struct sockaddr *)&sock->addr_in, &slen);

  pthread_mutex_lock(&udp_mutex);

  if (byte_read > 0) {
    for (i = 0; i < byte_read; i++) {
      p->rx_buf[p->rx_insert_idx] = buf[i];
      p->rx_insert_idx = (p->rx_insert_idx + 1) % UDP_RX_BUFFER_SIZE;
    }
  }

  pthread_mutex_unlock(&udp_mutex);
}

/**
 * Send a message
 */
void udp_send_message(struct udp_periph *p, long fd __attribute__((unused)))
{
  if (p == NULL) { return; }
  if (p->network == NULL) { return; }

  struct UdpSocket *sock = (struct UdpSocket *) p->network;

  if (p->tx_insert_idx > 0) {
    ssize_t bytes_sent = sendto(sock->sockfd, p->tx_buf, p->tx_insert_idx, MSG_DONTWAIT,
                                (struct sockaddr *)&sock->addr_out, sizeof(sock->addr_out));
    if (bytes_sent != p->tx_insert_idx) {
      if (bytes_sent < 0) {
        perror("udp_send_message failed");
      } else {
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
void udp_send_raw(struct udp_periph *p, long fd __attribute__((unused)), uint8_t *buffer, uint16_t size)
{
  if (p == NULL) { return; }
  if (p->network == NULL) { return; }

  struct UdpSocket *sock = (struct UdpSocket *) p->network;
  ssize_t test __attribute__((unused)) = sendto(sock->sockfd, buffer, size, MSG_DONTWAIT,
                                         (struct sockaddr *)&sock->addr_out, sizeof(sock->addr_out));
}

/**
 * check for new udp packets to receive or send.
 */
static void *udp_thread(void *data __attribute__((unused)))
{
  get_rt_prio(UDP_THREAD_PRIO);

  /* file descriptor list */
  fd_set socks_master;
  /* maximum file descriptor number */
  int fdmax = 0;

  /* clear the fd list */
  FD_ZERO(&socks_master);
  /* add used sockets */
  int fd __attribute__((unused));
#if USE_UDP0
  fd = ((struct UdpSocket *)udp0.network)->sockfd;
  FD_SET(fd, &socks_master);
  if (fd > fdmax) {
    fdmax = fd;
  }
#endif
#if USE_UDP1
  fd = ((struct UdpSocket *)udp1.network)->sockfd;
  FD_SET(fd, &socks_master);
  if (fd > fdmax) {
    fdmax = fd;
  }
#endif
#if USE_UDP2
  fd = ((struct UdpSocket *)udp2.network)->sockfd;
  FD_SET(fd, &socks_master);
  if (fd > fdmax) {
    fdmax = fd;
  }
#endif

  /* socks to be read, modified after each select */
  fd_set socks;

  while (1) {
    /* reset list of socks to check */
    socks = socks_master;

    if (select(fdmax + 1, &socks, NULL, NULL, NULL) < 0) {
      fprintf(stderr, "udp_thread: select failed!");
    } else {
#if USE_UDP0
      fd = ((struct UdpSocket *)udp0.network)->sockfd;
      if (FD_ISSET(fd, &socks)) {
        udp_receive(&udp0);
      }
#endif
#if USE_UDP1
      fd = ((struct UdpSocket *)udp1.network)->sockfd;
      if (FD_ISSET(fd, &socks)) {
        udp_receive(&udp1);
      }
#endif
#if USE_UDP2
      fd = ((struct UdpSocket *)udp2.network)->sockfd;
      if (FD_ISSET(fd, &socks)) {
        udp_receive(&udp2);
      }
#endif
    }
  }
  return 0;
}
