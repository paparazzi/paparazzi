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
 * @file arch/chibios/modules/dronecan/dronecan.h
 * Interface with dronecan using the Chibios can interfaces.
 * This uses multithreading and starts a transmit and receive thread per interface.
 */
#ifndef MODULES_DRONECAN_ARCH_H
#define MODULES_DRONECAN_ARCH_H

#include <hal.h>
#include <canard.h>
#include "dronecan_msgs.h"
#include "canard_chutils.h"
#include <string.h>

#ifndef MAX_CAN_NODES
#define MAX_CAN_NODES           128
#endif

struct node_activity {
  unsigned active : 1;
  uint64_t timestamp_usec;
};

/** dronecan interface structure */
struct dronecan_iface_t {
  CANDriver *can_driver;
  uint32_t can_baudrate;
  CANConfig can_cfg;
  uint8_t can_baudrate_mult;
  bool fdcan_operation;

  event_source_t tx_request;
  mutex_t mutex;

  void *thread_rx_wa;
  void *thread_tx_wa;
  void *thread_ns_wa;
  size_t thread_rx_wa_size;
  size_t thread_tx_wa_size;
  size_t thread_ns_wa_size;

  uint8_t node_id;
  struct uavcan_protocol_NodeStatus node_status;
  struct node_activity nodes_list[MAX_CAN_NODES];
  CanardInstance canard;
  uint8_t canard_memory_pool[1024 * 2];

  bool initialized;
};

/** Generic dronecan callback definition */
typedef void (*dronecan_callback)(struct dronecan_iface_t *iface, CanardRxTransfer *transfer);

/** Main dronecan event structure for registering/calling callbacks */
struct dronecan_event_t {
  CanardTransferType transfer_type;
  uint16_t data_type_id;
  uint64_t data_type_signature;
  dronecan_callback cb;
  struct dronecan_event_t *next;
};
typedef struct dronecan_event_t dronecan_event;

/** dronecan interfaces */
#if DRONECAN_USE_CAN1
extern struct dronecan_iface_t dronecan1;
#endif
#if DRONECAN_USE_CAN2
extern struct dronecan_iface_t dronecan2;
#endif

/** dronecan external functions */
void dronecan_bind(CanardTransferType transfer_type, uint16_t data_type_id, uint64_t data_type_signature, dronecan_event *ev, dronecan_callback cb);
void dronecan_broadcast(struct dronecan_iface_t *iface, CanardTxTransfer *transfer);
void dronecan_request_or_respond(struct dronecan_iface_t *iface, uint8_t destination_node_id,CanardTxTransfer *transfer);
void dronecan_init(void);

#endif /* MODULES_DRONECAN_ARCH_H */