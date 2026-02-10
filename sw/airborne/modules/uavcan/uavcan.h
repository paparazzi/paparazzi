/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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
 */
/**
 * @file arch/chibios/modules/uavcan/uavcan.h
 * Interface with uavcan using the Chibios can interfaces.
 * This uses multithreading and starts a transmit and receive thread per interface.
 */
#ifndef MODULES_UAVCAN_ARCH_H
#define MODULES_UAVCAN_ARCH_H

#include <canard.h>
#include <string.h>
#include "mcu_periph/can.h"
#include "modules/core/threads.h"
#include "utils/framed_ring_buffer.h"
#include "utils/kv_store.h"


#ifndef UAVCAN_TX_FIFO_SIZE
#define UAVCAN_TX_FIFO_SIZE 1024
#endif

#define UAVCAN_TID_STORE_CAPACITY   30

/** uavcan interface structure */
struct uavcan_iface_t {
  struct pprzaddr_can can_net;
  uint32_t can_baudrate;

  pprz_mutex_t rx_mutex;
  pprz_mutex_t tx_mutex;
  pprz_thread_t thread_tx;

  pprz_bsem_t bsem;
  
  uint8_t node_id;
  CanardInstance canard;
  uint8_t canard_memory_pool[1024 * 2];
  #if CANARD_ALLOCATE_SEM
  pprz_bsem_t allocator_bsem;
  #endif
  uint16_t nb_errors;

  //uint8_t transfer_id;  // TODO see comment: The Transfer ID value cannot be shared between requests that have different descriptors!
  uint32_t transfer_ids_keys[UAVCAN_TID_STORE_CAPACITY];
  uint8_t transfer_ids_values[UAVCAN_TID_STORE_CAPACITY];
  uint8_t transfer_ids_used[UAVCAN_TID_STORE_CAPACITY];
  kv_store_t transfer_ids_store;
  bool initialized;
};

/** Generic uavcan callback definition */
typedef void (*uavcan_callback)(struct uavcan_iface_t *iface, CanardRxTransfer *transfer);

/** Main uavcan event structure for registering/calling callbacks */
struct uavcan_event_t {
  uint16_t data_type_id;
  uint64_t data_type_signature;
  uavcan_callback cb;
  struct uavcan_event_t *next;
};
typedef struct uavcan_event_t uavcan_event;

/** uavcan interfaces */
#if UAVCAN_USE_CAN1
extern struct uavcan_iface_t uavcan1;
#endif
#if UAVCAN_USE_CAN2
extern struct uavcan_iface_t uavcan2;
#endif

/** uavcan external functions */
void uavcan_init(void);
void uavcan_reporting(void);
void uavcan_bind(uint16_t data_type_id, uint64_t data_type_signature, uavcan_event *ev, uavcan_callback cb);
void uavcan_transfer(struct uavcan_iface_t *iface, CanardTxTransfer* transfer);
void uavcan_request(struct uavcan_iface_t *iface, uint8_t destination_node_id, CanardTxTransfer* transfer);
void uavcan_broadcast(struct uavcan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id,
                      uint8_t priority, const void *payload, uint16_t payload_len);
void uavcan_response(struct uavcan_iface_t *iface, uint8_t destination_node_id, CanardTxTransfer* transfer);

#endif /* MODULES_UAVCAN_ARCH_H */
