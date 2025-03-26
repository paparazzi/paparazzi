/*
 * Copyright (C) 2020 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file arch/chibios/modules/uavcan/uavcan.c
 * Interface from actuators to ChibiOS CAN driver using UAVCan
 *
 */

#include "uavcan.h"
#include "mcu_periph/can.h"
#include "modules/core/threads.h"

#ifndef UAVCAN_NODE_ID
#define UAVCAN_NODE_ID    100
#endif

#ifndef UAVCAN_BAUDRATE
#define UAVCAN_BAUDRATE   1000000
#endif

static uavcan_event *uavcan_event_hd = NULL;

#if UAVCAN_USE_CAN1
#ifndef UAVCAN_CAN1_NODE_ID
#define UAVCAN_CAN1_NODE_ID UAVCAN_NODE_ID
#endif

#ifndef UAVCAN_CAN1_BAUDRATE
#define UAVCAN_CAN1_BAUDRATE UAVCAN_BAUDRATE
#endif


struct uavcan_iface_t uavcan1 = {
  .can_net = {.can_ifindex = 1},
  .can_baudrate = UAVCAN_CAN1_BAUDRATE,
  .node_id = UAVCAN_CAN1_NODE_ID,
  .transfer_id = 0,
  .initialized = false
};
#endif

#if UAVCAN_USE_CAN2
#ifndef UAVCAN_CAN2_NODE_ID
#define UAVCAN_CAN2_NODE_ID UAVCAN_NODE_ID
#endif

#ifndef UAVCAN_CAN2_BAUDRATE
#define UAVCAN_CAN2_BAUDRATE UAVCAN_BAUDRATE
#endif


struct uavcan_iface_t uavcan2 = {
  .can_net = {.can_ifindex = 2},
  .can_baudrate = UAVCAN_CAN2_BAUDRATE,
  .node_id = UAVCAN_CAN2_NODE_ID,
  .transfer_id = 0,
  .initialized = false
};
#endif


static void can_frame_cb(struct pprzcan_frame* rx_msg, UNUSED struct pprzaddr_can* src_addr, void* user_data) {
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)user_data;

  pprz_mtx_lock(&iface->mutex);

  CanardCANFrame rx_frame;
  memcpy(rx_frame.data, rx_msg->data, 8);
  rx_frame.data_len = rx_msg->len;
  if (rx_msg->can_id & CAN_FRAME_EFF) {
    rx_frame.id = CANARD_CAN_FRAME_EFF | (rx_msg->can_id & CAN_EID_MASK);
  } else {
    rx_frame.id = rx_msg->can_id & CAN_SID_MASK;
  }

  // Let canard handle the frame
  canardHandleRxFrame(&iface->canard, &rx_frame, rx_msg->timestamp);

  pprz_mtx_unlock(&iface->mutex);
}

/*
 * Transmitter thread.
 */
static void uavcan_tx(void* p)
{
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)p;
  uint8_t err_cnt = 0;

  while (true) {
    pprz_bsem_wait(&iface->bsem);

    pprz_mtx_lock(&iface->mutex);
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&iface->canard)) != NULL;) {
      struct pprzcan_frame tx_msg;
      memcpy(tx_msg.data, txf->data, 8);
      tx_msg.len = txf->data_len;
      tx_msg.can_id = (txf->id & CANARD_CAN_EXT_ID_MASK) | CAN_FRAME_EFF;

      if (!can_transmit_frame(&tx_msg, &iface->can_net)) {
        err_cnt = 0;
        canardPopTxQueue(&iface->canard);
      } else {
        // After 5 retries giveup and clean full queue
        if (err_cnt >= 5) {
          err_cnt = 0;
          while (canardPeekTxQueue(&iface->canard)) { canardPopTxQueue(&iface->canard); }
          continue;
        }

        // Timeout - just exit and try again later
        pprz_mtx_unlock(&iface->mutex);
        chThdSleepMilliseconds(++err_cnt);
        pprz_mtx_lock(&iface->mutex);
        continue;
      }
    }
    pprz_mtx_unlock(&iface->mutex);
  }
}

/**
 *  Whenever a valid and 'accepted' transfer is received
 */
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)ins->user_reference;

  // Go through all registered callbacks and call function callback if found
  for (uavcan_event *ev = uavcan_event_hd; ev; ev = ev->next) {
    if (transfer->data_type_id == ev->data_type_id) {
      ev->cb(iface, transfer);
    }
  }
}

/**
 * If we should accept this transfer
 */
static bool shouldAcceptTransfer(const CanardInstance *ins __attribute__((unused)),
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type __attribute__((unused)),
                                 uint8_t source_node_id __attribute__((unused)))
{

  // Go through all registered callbacks and return signature if found
  for (uavcan_event *ev = uavcan_event_hd; ev; ev = ev->next) {
    if (data_type_id == ev->data_type_id) {
      *out_data_type_signature = ev->data_type_signature;
      return true;
    }
  }
  // No callback found return
  return false;
}


/**
 * Initialize uavcan interface
 */
static void uavcanInitIface(struct uavcan_iface_t *iface)
{
  pprz_mtx_init(&iface->mutex);
  pprz_bsem_init(&iface->bsem, true);

  // Initialize canard
  canardInit(&iface->canard, iface->canard_memory_pool, sizeof(iface->canard_memory_pool),
             onTransferReceived, shouldAcceptTransfer, iface);
  // Update the uavcan node ID
  canardSetLocalNodeID(&iface->canard, iface->node_id);

  // Start the receiver and transmitter thread
  can_register_callback(can_frame_cb, &iface->can_net, (void *)iface);

  if(!pprz_thread_create(&iface->thread_tx, 2048, "uavcan_tx", NORMALPRIO+7, uavcan_tx, (void *)iface)) {
    iface->initialized = true;
  }
}

/**
 * Initialize all uavcan interfaces
 */
void uavcan_init(void)
{
#if UAVCAN_USE_CAN1
  uavcanInitIface(&uavcan1);
#endif
#if UAVCAN_USE_CAN2
  uavcanInitIface(&uavcan2);
#endif
}

/**
 * Bind to a receiving message from uavcan
 */
void uavcan_bind(uint16_t data_type_id, uint64_t data_type_signature, uavcan_event *ev, uavcan_callback cb)
{
  // Configure the event
  ev->data_type_id = data_type_id;
  ev->data_type_signature = data_type_signature;
  ev->cb = cb;
  ev->next = uavcan_event_hd;

  // Switch the head
  uavcan_event_hd = ev;
}

/**
 * Broadcast an uavcan message to a specific interface
 */
void uavcan_broadcast(struct uavcan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id,
                      uint8_t priority, const void *payload,
                      uint16_t payload_len)
{
  if (!iface->initialized) { return; }
  pprz_mtx_lock(&iface->mutex);
  canardBroadcast(&iface->canard,
                  data_type_signature,
                  data_type_id, &iface->transfer_id,
                  priority, payload, payload_len);
  pprz_mtx_unlock(&iface->mutex);
  pprz_bsem_signal(&iface->bsem);
}
