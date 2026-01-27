/*
 * device over uavcan tunnel.
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

#include "uavcan_tunnel.h"
#include "uavcan/uavcan.h"
#include <uavcan.tunnel.Protocol.h>

#if !defined(UAVCAN_TUNNEL_IFACE) && USE_CAN1
#define UAVCAN_TUNNEL_IFACE uavcan1
#endif

#if !defined(UAVCAN_TUNNEL_IFACE) && USE_CAN2
#define UAVCAN_TUNNEL_IFACE uavcan2
#endif

#if !defined(UAVCAN_TUNNEL_PROTOCOL)
#define UAVCAN_TUNNEL_PROTOCOL 3
#endif

#if !defined(UAVCAN_TUNNEL_CHANNEL_ID)
#define UAVCAN_TUNNEL_CHANNEL_ID 0
#endif

struct uavcan_tunnel_periph uavcan_tunnel0;


// Serial device functions
static int      uavcan_tunnel_check_free_space(struct uavcan_tunnel_periph* up, long *fd, uint16_t len);
static void     uavcan_tunnel_put_byte(struct uavcan_tunnel_periph* up, long fd, uint8_t c);
static void     uavcan_tunnel_put_buffer(struct uavcan_tunnel_periph* up, long fd, const uint8_t *data, uint16_t len);
static void     uavcan_tunnel_send_message(struct uavcan_tunnel_periph* up, long fd);
static int      uavcan_tunnel_char_available(struct uavcan_tunnel_periph* up);
static uint8_t  uavcan_tunnel_get_byte(struct uavcan_tunnel_periph* up);
void uavcan_tunnel_set_baudrate(struct uavcan_tunnel_periph* up, uint32_t baudrate);

static void uavcan_tunnel_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer);

void uavcan_tunnel_init_periph(struct uavcan_tunnel_periph* ucdl, uint8_t channel_id);

void uavcan_tunnel_init(void) {
  uavcan_tunnel_init_periph(&uavcan_tunnel0, UAVCAN_TUNNEL_CHANNEL_ID);
}

void uavcan_tunnel_init_periph(struct uavcan_tunnel_periph* ucdl, uint8_t channel_id) {
  ucdl->channel_id = channel_id;
  ucdl->iface = &UAVCAN_TUNNEL_IFACE;
  ring_buffer_init(&ucdl->tx_rb, ucdl->_tbuf, TX_BUFFER_SIZE);
  ring_buffer_init(&ucdl->rx_rb, ucdl->_rbuf, RX_BUFFER_SIZE);
  pprz_mtx_init(&ucdl->tx_mtx);
  pprz_mtx_init(&ucdl->rx_mtx);

  uavcan_bind(UAVCAN_TUNNEL_BROADCAST_ID, UAVCAN_TUNNEL_BROADCAST_SIGNATURE, &ucdl->tunnel_brd_ev, &uavcan_tunnel_cb);

  // setup device
  ucdl->device.check_free_space = (check_free_space_t)uavcan_tunnel_check_free_space;
  ucdl->device.put_byte = (put_byte_t)uavcan_tunnel_put_byte;
  ucdl->device.put_buffer = (put_buffer_t)uavcan_tunnel_put_buffer;
  ucdl->device.send_message = (send_message_t)uavcan_tunnel_send_message;
  ucdl->device.char_available = (char_available_t)uavcan_tunnel_char_available;
  ucdl->device.get_byte = (get_byte_t)uavcan_tunnel_get_byte;
  ucdl->device.set_baudrate = (set_baudrate_t)uavcan_tunnel_set_baudrate;
  ucdl->device.periph = ucdl;
}

static void uavcan_tunnel_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer) {
  (void)iface;
  struct uavcan_tunnel_Broadcast utb_msg;
  if(uavcan_tunnel_Broadcast_decode(transfer, &utb_msg)) {
    // decode error
    return;
  }

  if(utb_msg.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL) {
    return;
  }
  
  if(uavcan_tunnel0.channel_id == utb_msg.channel_id) {
    if(pprz_mtx_trylock(&uavcan_tunnel0.rx_mtx)==0) {
      ring_buffer_write(&uavcan_tunnel0.rx_rb, utb_msg.buffer.data, utb_msg.buffer.len);
      pprz_mtx_unlock(&uavcan_tunnel0.rx_mtx);
    }
  }
}

/**
 * @return free_space if free_space > len, else 0.
 * That's not logic...
 */
static int uavcan_tunnel_check_free_space(struct uavcan_tunnel_periph* up, long *fd, uint16_t len) {
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

static void uavcan_tunnel_put_byte(struct uavcan_tunnel_periph* up, long fd, uint8_t c) {
  uavcan_tunnel_put_buffer(up, fd, (const uint8_t*)&c, 1);
}

static void uavcan_tunnel_put_buffer(struct uavcan_tunnel_periph* up, long fd, const uint8_t *data, uint16_t len) {
  if(fd == 0) {
    // if fd is zero, assume the driver is not already locked
    pprz_mtx_lock(&up->tx_mtx);
  }

  ring_buffer_write(&up->tx_rb, data, len);

  if(fd == 0) {
    // send immediately. mutex is already locked, so set flag.
    uavcan_tunnel_send_message(up, 1);
  }
}

static void uavcan_tunnel_send_message(struct uavcan_tunnel_periph* up, long fd) {
  struct uavcan_tunnel_Broadcast msg;
  uint8_t buffer[UAVCAN_TUNNEL_BROADCAST_MAX_SIZE];
  msg.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL;

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


static int uavcan_tunnel_char_available(struct uavcan_tunnel_periph* up) {
  int available = 0;
  if(pprz_mtx_lock(&up->rx_mtx) == 0) {
    available = (int)ring_buffer_available(&up->rx_rb);
    pprz_mtx_unlock(&up->rx_mtx);
  }
  return available;
}

static uint8_t uavcan_tunnel_get_byte(struct uavcan_tunnel_periph* up) {
  uint8_t c;
  if(pprz_mtx_lock(&up->rx_mtx) == 0) {
    ring_buffer_read(&up->rx_rb, &c, 1);
    pprz_mtx_unlock(&up->rx_mtx);
  }
  return c;
}


void uavcan_tunnel_set_baudrate(struct uavcan_tunnel_periph* up __attribute__((unused)), uint32_t baudrate __attribute__((unused))) {

}