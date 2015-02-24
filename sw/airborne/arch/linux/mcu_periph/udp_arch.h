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

/** @file arch/linux/mcu_periph/udp_arch.h
 * linux UDP handling
 */

#ifndef UDP_ARCH_H
#define UDP_ARCH_H

#include "mcu_periph/udp.h"
#include <sys/socket.h>
#include <arpa/inet.h>

struct UdpNetwork {
  int socket_in;
  int socket_out;
  struct sockaddr_in addr_in;
  struct sockaddr_in addr_out;
};

/**
 * Create UDP network (in/out sockets).
 * @param[out] network   pointer to already allocated UdpNetwork struct
 * @param[in]  host      hostname/address
 * @param[in]  port_out  output port
 * @param[in]  port_in   input port
 * @param[in]  broadcast if TRUE enable broadcasting
 */
extern void udp_create_network(struct UdpNetwork *network, char *host, int port_out, int port_in, bool_t broadcast);

#endif /* UDP_ARCH_H */
