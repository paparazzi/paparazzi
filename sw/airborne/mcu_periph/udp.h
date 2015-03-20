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

/** \file mcu_periph/udp.h
 *  \brief arch independent UDP API
 *
 */

#ifndef MCU_PERIPH_UDP_H
#define MCU_PERIPH_UDP_H

#include "std.h"
#include "mcu_periph/udp_arch.h"
#include "mcu_periph/link_device.h"

#define UDP_RX_BUFFER_SIZE 256
#define UDP_TX_BUFFER_SIZE 256

struct udp_periph {
  /** Receive buffer */
  uint8_t rx_buf[UDP_RX_BUFFER_SIZE];
  uint16_t rx_insert_idx;
  uint16_t rx_extract_idx;
  /** Transmit buffer */
  uint8_t tx_buf[UDP_TX_BUFFER_SIZE];
  uint16_t tx_insert_idx;
  /** UDP network */
  void *network;
  /** Generic device interface */
  struct link_device device;
};

extern void     udp_periph_init(struct udp_periph *p, char *host, int port_out, int port_in, bool_t broadcast);
extern bool_t   udp_check_free_space(struct udp_periph *p, uint8_t len);
extern void     udp_transmit(struct udp_periph *p, uint8_t data);
extern uint16_t udp_char_available(struct udp_periph *p);
extern uint8_t  udp_getch(struct udp_periph *p);
extern void     udp_event(void);
extern void     udp_arch_periph_init(struct udp_periph *p, char *host, int port_out, int port_in, bool_t broadcast);
extern void     udp_send_message(struct udp_periph *p);
extern void     udp_send_raw(struct udp_periph *p, uint8_t *buffer, uint16_t size);
extern void     udp_receive(struct udp_periph *p);

#if USE_UDP0
extern struct udp_periph udp0;

#ifndef UDP0_HOST
#define UDP0_HOST "127.0.0.1"
#endif

#ifndef UDP0_PORT_OUT
#define UDP0_PORT_OUT 4242
#endif

#ifndef UDP0_PORT_IN
#define UDP0_PORT_IN 4243
#endif

#ifndef UDP0_BROADCAST
#define UDP0_BROADCAST FALSE
#endif

#define UDP0Init() udp_periph_init(&udp0, UDP0_HOST, UDP0_PORT_OUT, UDP0_PORT_IN, UDP0_BROADCAST)
#endif // USE_UDP0

#if USE_UDP1
extern struct udp_periph udp1;

#ifndef UDP1_HOST
#define UDP1_HOST "127.0.0.1"
#endif

#ifndef UDP1_PORT_OUT
#define UDP1_PORT_OUT 4244
#endif

#ifndef UDP1_PORT_IN
#define UDP1_PORT_IN 4245
#endif

#ifndef UDP1_BROADCAST
#define UDP1_BROADCAST FALSE
#endif

#define UDP1Init() udp_periph_init(&udp1, UDP1_HOST, UDP1_PORT_OUT, UDP1_PORT_IN, UDP1_BROADCAST)
#endif // USE_UDP1

#if USE_UDP2
extern struct udp_periph udp2;

#ifndef UDP2_HOST
#define UDP2_HOST "127.0.0.1"
#endif

#ifndef UDP2_PORT_OUT
#define UDP2_PORT_OUT 4246
#endif

#ifndef UDP2_PORT_IN
#define UDP2_PORT_IN 4247
#endif

#ifndef UDP2_BROADCAST
#define UDP2_BROADCAST FALSE
#endif

#define UDP2Init() udp_periph_init(&udp2, UDP2_HOST, UDP2_PORT_OUT, UDP2_PORT_IN, UDP2_BROADCAST)
#endif // USE_UDP2

#endif /* MCU_PERIPH_UDP_H */
