/*
 * device over uavcan tunnel.
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

#include "tunnel_uavcan.h"
#include "uavcan/uavcan.h"
#include <uavcan.tunnel.Protocol.h>

#if !defined(TUNNEL_UAVCAN_IFACE) && USE_CAN1
#define TUNNEL_UAVCAN_IFACE uavcan1
#endif

#if !defined(TUNNEL_UAVCAN_IFACE) && USE_CAN2
#define TUNNEL_UAVCAN_IFACE uavcan2
#endif

#if !defined(TUNNEL_UAVCAN_PROTOCOL)
#define TUNNEL_UAVCAN_PROTOCOL 3
#endif

#if !defined(TUNNEL_UAVCAN_CHANNEL_ID)
#define TUNNEL_UAVCAN_CHANNEL_ID 0
#endif

struct tunnel_uavcan_periph tunnel_uavcan0;


// Serial device functions
static int      tunnel_uavcan_check_free_space(struct tunnel_uavcan_periph* up, long *fd, uint16_t len);
static void     tunnel_uavcan_put_byte(struct tunnel_uavcan_periph* up, long fd, uint8_t c);
static void     tunnel_uavcan_put_buffer(struct tunnel_uavcan_periph* up, long fd, const uint8_t *data, uint16_t len);
static void     tunnel_uavcan_send_message(struct tunnel_uavcan_periph* up, long fd);
static int      tunnel_uavcan_char_available(struct tunnel_uavcan_periph* up);
static uint8_t  tunnel_uavcan_get_byte(struct tunnel_uavcan_periph* up);
void tunnel_uavcan_set_baudrate(struct tunnel_uavcan_periph* up, uint32_t baudrate);

static void tunnel_uavcan_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer);

void tunnel_uavcan_init_periph(struct tunnel_uavcan_periph* ucdl, uint8_t channel_id);

void tunnel_uavcan_init(void) {
  tunnel_uavcan_init_periph(&tunnel_uavcan0, TUNNEL_UAVCAN_CHANNEL_ID);
}

void tunnel_uavcan_init_periph(struct tunnel_uavcan_periph* ucdl, uint8_t channel_id) {
  ucdl->channel_id = channel_id;
  ucdl->iface = &TUNNEL_UAVCAN_IFACE;
  ring_buffer_init(&ucdl->tx_rb, ucdl->_tbuf, TX_BUFFER_SIZE);
  ring_buffer_init(&ucdl->rx_rb, ucdl->_rbuf, RX_BUFFER_SIZE);
  pprz_mtx_init(&ucdl->tx_mtx);
  pprz_mtx_init(&ucdl->rx_mtx);

  uavcan_bind(UAVCAN_TUNNEL_BROADCAST_ID, UAVCAN_TUNNEL_BROADCAST_SIGNATURE, &ucdl->tunnel_brd_ev, &tunnel_uavcan_cb);

  // setup device
  ucdl->device.check_free_space = (check_free_space_t)tunnel_uavcan_check_free_space;
  ucdl->device.put_byte = (put_byte_t)tunnel_uavcan_put_byte;
  ucdl->device.put_buffer = (put_buffer_t)tunnel_uavcan_put_buffer;
  ucdl->device.send_message = (send_message_t)tunnel_uavcan_send_message;
  ucdl->device.char_available = (char_available_t)tunnel_uavcan_char_available;
  ucdl->device.get_byte = (get_byte_t)tunnel_uavcan_get_byte;
  ucdl->device.set_baudrate = (set_baudrate_t)tunnel_uavcan_set_baudrate;
  ucdl->device.periph = ucdl;
}

static void tunnel_uavcan_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer) {
  (void)iface;
  struct uavcan_tunnel_Broadcast utb_msg;
  if(uavcan_tunnel_Broadcast_decode(transfer, &utb_msg)) {
    // decode error
    return;
  }

  if(utb_msg.protocol.protocol != TUNNEL_UAVCAN_PROTOCOL) {
    return;
  }
  
  if(tunnel_uavcan0.channel_id == utb_msg.channel_id) {
    if(pprz_mtx_trylock(&tunnel_uavcan0.rx_mtx)==0) {
      ring_buffer_write(&tunnel_uavcan0.rx_rb, utb_msg.buffer.data, utb_msg.buffer.len);
      pprz_mtx_unlock(&tunnel_uavcan0.rx_mtx);
    }
  }
}

/**
 * @return free_space if free_space > len, else 0.
 * That's not logic...
 */
static int tunnel_uavcan_check_free_space(struct tunnel_uavcan_periph* up, long *fd, uint16_t len) {
  int free_space = 0;
  if(pprz_mtx_lock(&up->tx_mtx) == 0) {
    free_space = (int)ring_buffer_free_space(&up->tx_rb);
    if(len < free_space) {
      // keep the device locked and set the flag in fd.
      *fd = 1;
    } else {
      // no space available, unlock.
      free_space = 0;
      pprz_mtx_unlock(&up->tx_mtx);
    }
    
  }
  return free_space;
}

static void tunnel_uavcan_put_byte(struct tunnel_uavcan_periph* up, long fd, uint8_t c) {
  tunnel_uavcan_put_buffer(up, fd, (const uint8_t*)&c, 1);
}

static void tunnel_uavcan_put_buffer(struct tunnel_uavcan_periph* up, long fd, const uint8_t *data, uint16_t len) {
  if(fd == 0) {
    // if fd is zero, assume the driver is not already locked
    pprz_mtx_lock(&up->tx_mtx);
  }

  ring_buffer_write(&up->tx_rb, data, len);

  if(fd == 0) {
    // send immediately. mutex is already locked, so set flag.
    tunnel_uavcan_send_message(up, 1);
  }
}

static void tunnel_uavcan_send_message(struct tunnel_uavcan_periph* up, long fd) {
  struct uavcan_tunnel_Broadcast msg;
  uint8_t buffer[UAVCAN_TUNNEL_BROADCAST_MAX_SIZE];
  msg.protocol.protocol = TUNNEL_UAVCAN_PROTOCOL;

  if(fd == 0) {
    // flag not set, lock and unlock
    pprz_mtx_lock(&up->tx_mtx);
  }

  // Empty the buffer
  while(ring_buffer_available(&up->tx_rb)) {
    size_t len = ring_buffer_read(&up->tx_rb, msg.buffer.data, sizeof(msg.buffer.data));
    msg.buffer.len = len;
    msg.channel_id = up->channel_id;

    // Encode the message
    uint32_t total_size = uavcan_tunnel_Broadcast_encode(&msg, buffer);
    // Then send it
    uavcan_broadcast(up->iface, UAVCAN_TUNNEL_BROADCAST_SIGNATURE, UAVCAN_TUNNEL_BROADCAST_ID, CANARD_TRANSFER_PRIORITY_MEDIUM, buffer, total_size);
  }

  pprz_mtx_unlock(&up->tx_mtx);
}


static int tunnel_uavcan_char_available(struct tunnel_uavcan_periph* up) {
  int available = 0;
  if(pprz_mtx_lock(&up->rx_mtx) == 0) {
    available = (int)ring_buffer_available(&up->rx_rb);
    pprz_mtx_unlock(&up->rx_mtx);
  }
  return available;
}

static uint8_t tunnel_uavcan_get_byte(struct tunnel_uavcan_periph* up) {
  uint8_t c;
  if(pprz_mtx_lock(&up->rx_mtx) == 0) {
    ring_buffer_read(&up->rx_rb, &c, 1);
    pprz_mtx_unlock(&up->rx_mtx);
  }
  return c;
}


void tunnel_uavcan_set_baudrate(struct tunnel_uavcan_periph* up __attribute__((unused)), uint32_t baudrate __attribute__((unused))) {

}