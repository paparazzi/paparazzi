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
#include <string.h>
#include <time.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include "rt_priority.h"

#ifndef UDP_THREAD_PRIO
#define UDP_THREAD_PRIO 10
#endif

static void *udp_thread(void *data __attribute__((unused)));
static pthread_mutex_t udp_mutex = PTHREAD_MUTEX_INITIALIZER;

/*
 * Host "auto": find the GCS computers instead of sending to one fixed address.
 * Every address that sends to the input port is a GCS (link pings every 5 s) and gets the messages unicast
 * (Wi-Fi acknowledges and retries unicast frames, not broadcast ones). Without a GCS, and for one send per
 * second, the messages are broadcast instead to the broadcast address of every network interface, so a GCS
 * started later finds the aircraft and no GCS gets a message twice.
 */
#define UDP_AUTO_HOST           "auto"
#define UDP_AUTO_MAX_GCS        8
#define UDP_AUTO_MAX_BCAST      8
#define UDP_AUTO_GCS_TIMEOUT_MS 15000
#define UDP_AUTO_BCAST_PERIOD   1000
#define UDP_AUTO_IF_REFRESH_MS  5000

struct UdpAuto {
  bool enabled;
  uint16_t port_out;
  struct sockaddr_in gcs[UDP_AUTO_MAX_GCS];
  uint32_t gcs_seen_ms[UDP_AUTO_MAX_GCS];   ///< 0: free slot
  struct sockaddr_in bcast[UDP_AUTO_MAX_BCAST];
  int nb_bcast;
  uint32_t bcast_refresh_ms;
  uint32_t last_bcast_ms;
};

/* The socket comes first, so p->network can still be used as a struct UdpSocket */
struct UdpArch {
  struct UdpSocket sock;
  struct UdpAuto au;
};

static uint32_t udp_time_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint32_t ms = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  return ms ? ms : 1;
}

/** Broadcast address of every interface that is up (not loopback) */
static void udp_auto_refresh_bcast(struct UdpAuto *au)
{
  struct ifaddrs *ifas;
  au->nb_bcast = 0;
  if (getifaddrs(&ifas) == 0) {
    for (struct ifaddrs *i = ifas; i != NULL && au->nb_bcast < UDP_AUTO_MAX_BCAST; i = i->ifa_next) {
      if (i->ifa_addr == NULL || i->ifa_addr->sa_family != AF_INET || i->ifa_broadaddr == NULL ||
          !(i->ifa_flags & IFF_UP) || !(i->ifa_flags & IFF_BROADCAST) || (i->ifa_flags & IFF_LOOPBACK)) {
        continue;
      }
      memcpy(&au->bcast[au->nb_bcast], i->ifa_broadaddr, sizeof(struct sockaddr_in));
      au->bcast[au->nb_bcast].sin_port = htons(au->port_out);
      au->nb_bcast++;
    }
    freeifaddrs(ifas);
  }
  if (au->nb_bcast == 0) {
    memset(&au->bcast[0], 0, sizeof(au->bcast[0]));
    au->bcast[0].sin_family = AF_INET;
    au->bcast[0].sin_addr.s_addr = htonl(INADDR_BROADCAST);
    au->bcast[0].sin_port = htons(au->port_out);
    au->nb_bcast = 1;
  }
}

/** Remember the sender of a datagram as a GCS (called with the udp mutex) */
static void udp_auto_gcs_seen(struct UdpAuto *au, struct sockaddr_in *from)
{
  uint32_t now = udp_time_ms();
  int slot = -1;
  for (int i = 0; i < UDP_AUTO_MAX_GCS; i++) {
    if (au->gcs_seen_ms[i] != 0 && au->gcs[i].sin_addr.s_addr == from->sin_addr.s_addr &&
        au->gcs[i].sin_port == from->sin_port) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    // a free or expired slot, else the oldest one
    for (int i = 0; i < UDP_AUTO_MAX_GCS; i++) {
      if (slot < 0 || au->gcs_seen_ms[i] == 0 || now - au->gcs_seen_ms[i] > UDP_AUTO_GCS_TIMEOUT_MS ||
          au->gcs_seen_ms[i] < au->gcs_seen_ms[slot]) {
        slot = i;
      }
      if (au->gcs_seen_ms[slot] == 0 || now - au->gcs_seen_ms[slot] > UDP_AUTO_GCS_TIMEOUT_MS) {
        break;
      }
    }
    au->gcs[slot] = *from;
  }
  au->gcs_seen_ms[slot] = now;
}

/** Send to the GCS computers found, or broadcast (see UdpAuto) */
static void udp_auto_send(struct UdpArch *ua, uint8_t *buffer, uint16_t size)
{
  struct UdpAuto *au = &ua->au;
  uint32_t now = udp_time_ms();
  struct sockaddr_in to[UDP_AUTO_MAX_GCS];
  int nb = 0;

  pthread_mutex_lock(&udp_mutex);
  for (int i = 0; i < UDP_AUTO_MAX_GCS; i++) {
    if (au->gcs_seen_ms[i] != 0 && now - au->gcs_seen_ms[i] <= UDP_AUTO_GCS_TIMEOUT_MS) {
      to[nb++] = au->gcs[i];
    }
  }
  pthread_mutex_unlock(&udp_mutex);

  if (nb == 0 || now - au->last_bcast_ms >= UDP_AUTO_BCAST_PERIOD) {
    if (au->bcast_refresh_ms == 0 || now - au->bcast_refresh_ms >= UDP_AUTO_IF_REFRESH_MS) {
      udp_auto_refresh_bcast(au);
      au->bcast_refresh_ms = now;
    }
    au->last_bcast_ms = now;
    for (int i = 0; i < au->nb_bcast; i++) {
      sendto(ua->sock.sockfd, buffer, size, MSG_DONTWAIT, (struct sockaddr *)&au->bcast[i], sizeof(au->bcast[i]));
    }
  } else {
    for (int i = 0; i < nb; i++) {
      sendto(ua->sock.sockfd, buffer, size, MSG_DONTWAIT, (struct sockaddr *)&to[i], sizeof(to[i]));
    }
  }
}

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
  struct UdpArch *ua = calloc(1, sizeof(struct UdpArch));
  ua->au.enabled = (strcmp(host, UDP_AUTO_HOST) == 0);
  ua->au.port_out = port_out;
  if (ua->au.enabled) {
    // broadcast until a GCS is found
    udp_socket_create(&ua->sock, "255.255.255.255", port_out, port_in, true);
  } else {
    udp_socket_create(&ua->sock, host, port_out, port_in, broadcast);
  }
  p->network = (void *)ua;
}

/**
 * Get number of bytes available in receive buffer.
 * @param p pointer to UDP peripheral
 * @return number of bytes available in receive buffer
 */
int udp_char_available(struct udp_periph *p)
{
  pthread_mutex_lock(&udp_mutex);
  int available = p->rx_insert_idx - p->rx_extract_idx;
  if (available < 0) {
    available += UDP_RX_BUFFER_SIZE;
  }
  pthread_mutex_unlock(&udp_mutex);
  return available;
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
    struct UdpArch *ua = (struct UdpArch *) p->network;
    if (ua->au.enabled) {
      udp_auto_gcs_seen(&ua->au, &sock->addr_in);
    }
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

  if (p->tx_insert_idx > 0 && ((struct UdpArch *) p->network)->au.enabled) {
    udp_auto_send((struct UdpArch *) p->network, p->tx_buf, p->tx_insert_idx);
    p->tx_insert_idx = 0;
  } else if (p->tx_insert_idx > 0) {
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

  if (((struct UdpArch *) p->network)->au.enabled) {
    udp_auto_send((struct UdpArch *) p->network, buffer, size);
    return;
  }
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
