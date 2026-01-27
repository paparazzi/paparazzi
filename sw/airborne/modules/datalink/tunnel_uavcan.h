/*
 * device over uavcan tunnel.
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

#pragma once

#include "pprzlink/pprzlink_device.h"
#include "uavcan.tunnel.Broadcast.h"
#include "utils/ring_buffer.h"
#include "core/threads.h"
#include "uavcan/uavcan.h"

#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 512

struct tunnel_uavcan_periph {
  uint8_t channel_id;
  uint8_t _tbuf[TX_BUFFER_SIZE];
  ring_buffer_t tx_rb;
  uint8_t _rbuf[RX_BUFFER_SIZE];
  ring_buffer_t rx_rb;

  pprz_mutex_t tx_mtx;
  pprz_mutex_t rx_mtx;
  uavcan_event tunnel_brd_ev;

  struct uavcan_iface_t *iface;

  /** Generic device interface */
  struct link_device device;
};

extern struct tunnel_uavcan_periph tunnel_uavcan0;

void tunnel_uavcan_init(void);
