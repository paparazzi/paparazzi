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

static THD_WORKING_AREA(uavcan1_rx_wa, 1024*2);
static THD_WORKING_AREA(uavcan1_tx_wa, 1024*2);

struct uavcan_iface_t uavcan1 = {
  .can_driver = &CAND1,
  .can_cfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(14) | CAN_BTR_BRP((STM32_PCLK1/18)/UAVCAN_CAN1_BAUDRATE - 1)
  },
  .thread_rx_wa = uavcan1_rx_wa,
  .thread_rx_wa_size = sizeof(uavcan1_rx_wa),
  .thread_tx_wa = uavcan1_tx_wa,
  .thread_tx_wa_size = sizeof(uavcan1_tx_wa),
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

static THD_WORKING_AREA(uavcan2_rx_wa, 1024*2);
static THD_WORKING_AREA(uavcan2_tx_wa, 1024*2);

struct uavcan_iface_t uavcan2 = {
  .can_driver = &CAND2,
  .can_cfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(14) | CAN_BTR_BRP((STM32_PCLK1/18)/UAVCAN_CAN2_BAUDRATE - 1)
  },
  .thread_rx_wa = uavcan2_rx_wa,
  .thread_rx_wa_size = sizeof(uavcan2_rx_wa),
  .thread_tx_wa = uavcan2_tx_wa,
  .thread_tx_wa_size = sizeof(uavcan2_tx_wa),
  .node_id = UAVCAN_CAN2_NODE_ID,
  .transfer_id = 0,
  .initialized = false
};
#endif

/*
 * Receiver thread.
 */
static THD_FUNCTION(uavcan_rx, p) {
  event_listener_t el;
  CANRxFrame rx_msg;
  CanardCANFrame rx_frame;
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)p;

  chRegSetThreadName("uavcan_rx");
  chEvtRegister(&iface->can_driver->rxfull_event, &el, EVENT_MASK(0));
  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0)
      continue;
    chMtxLock(&iface->mutex);

    // Wait until a CAN message is received
    while (canReceive(iface->can_driver, CAN_ANY_MAILBOX, &rx_msg, TIME_IMMEDIATE) == MSG_OK) {
      // Process message.
      const uint32_t timestamp = TIME_I2US(chVTGetSystemTimeX());
      memcpy(rx_frame.data, rx_msg.data8, 8);
      rx_frame.data_len = rx_msg.DLC;
      if(rx_msg.IDE) {
        rx_frame.id = CANARD_CAN_FRAME_EFF | rx_msg.EID;
      } else {
        rx_frame.id = rx_msg.SID;
      }
 
      // Let canard handle the frame
      canardHandleRxFrame(&iface->canard, &rx_frame, timestamp);
    }
    chMtxUnlock(&iface->mutex);
  }
  chEvtUnregister(&iface->can_driver->rxfull_event, &el);
}

/*
 * Transmitter thread.
 */
static THD_FUNCTION(uavcan_tx, p) {
  event_listener_t txc, txe, txr;
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)p;
  uint8_t err_cnt = 0;

  chRegSetThreadName("uavcan_tx");
  chEvtRegister(&iface->can_driver->txempty_event, &txc, EVENT_MASK(0));
  chEvtRegister(&iface->can_driver->error_event, &txe, EVENT_MASK(1));
  chEvtRegister(&iface->tx_request, &txr, EVENT_MASK(2));

  while (true) {
    eventmask_t evts = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));
    // Succesfull transmit
    if (evts == 0)
      continue;

    // Transmit error
    if(evts & EVENT_MASK(1))
    {
      chEvtGetAndClearFlags(&txe);
      continue;
    }

    chMtxLock(&iface->mutex);
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&iface->canard)) != NULL;) {
      CANTxFrame tx_msg;
      tx_msg.DLC = txf->data_len;
      memcpy(tx_msg.data8, txf->data, 8);
      tx_msg.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
      tx_msg.IDE = CAN_IDE_EXT;
      tx_msg.RTR = CAN_RTR_DATA;
      if (canTransmit(iface->can_driver, CAN_ANY_MAILBOX, &tx_msg, TIME_IMMEDIATE) == MSG_OK) {
        err_cnt = 0;
        canardPopTxQueue(&iface->canard);
      } else {
        // After 5 retries giveup
        if(err_cnt >= 5) {
          err_cnt = 0;
          canardPopTxQueue(&iface->canard);
          continue;
        }

        // Timeout - just exit and try again later
        chMtxUnlock(&iface->mutex);
        err_cnt++;
        canardPopTxQueue(&iface->canard); //FIXME (This needs to be here, don't know why)
        chThdSleepMilliseconds(err_cnt * 5);
        chMtxLock(&iface->mutex);
        continue;
      }
    }
    chMtxUnlock(&iface->mutex);
  }
}

/**
 *  Whenever a valid and 'accepted' transfer is received
 */
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer) {
  struct uavcan_iface_t *iface = (struct uavcan_iface_t *)ins->user_reference;

  // Go through all registered callbacks and call function callback if found
  for(uavcan_event *ev = uavcan_event_hd; ev; ev = ev->next) {
    if(transfer->data_type_id == ev->data_type_id)
      ev->cb(iface, transfer);
  }
}

/**
 * If we should accept this transfer
 */
static bool shouldAcceptTransfer(const CanardInstance* ins __attribute__((unused)),
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type __attribute__((unused)),
                                 uint8_t source_node_id __attribute__((unused))) {

  // Go through all registered callbacks and return signature if found
  for(uavcan_event *ev = uavcan_event_hd; ev; ev = ev->next) {
    if(data_type_id == ev->data_type_id) {
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
static void uavcanInitIface(struct uavcan_iface_t *iface) {
  // Initialize mutexes/events for multithread locking
  chMtxObjectInit(&iface->mutex);
  chEvtObjectInit(&iface->tx_request);

  // Initialize canard
  canardInit(&iface->canard, iface->canard_memory_pool, sizeof(iface->canard_memory_pool),
    onTransferReceived, shouldAcceptTransfer, iface);
  // Update the uavcan node ID
  canardSetLocalNodeID(&iface->canard, iface->node_id);

  // Start the can interface
  canStart(iface->can_driver, &iface->can_cfg);

  // Start the receiver and transmitter thread
  chThdCreateStatic(iface->thread_rx_wa, iface->thread_rx_wa_size, NORMALPRIO + 8, uavcan_rx, (void*)iface);
  chThdCreateStatic(iface->thread_tx_wa, iface->thread_tx_wa_size, NORMALPRIO + 7, uavcan_tx, (void*)iface);
  iface->initialized = true;
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
  ev->data_type_id = data_type_id,
  ev->data_type_signature = data_type_signature,
  ev->cb = cb,
  ev->next = uavcan_event_hd;

  // Switch the head
  uavcan_event_hd = ev;
}

/**
 * Broadcast an uavcan message to a specific interface
 */
void uavcan_broadcast(struct uavcan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id, uint8_t priority, const void* payload,
                        uint16_t payload_len) {
  if(!iface->initialized) return;

  chMtxLock(&iface->mutex);
  canardBroadcast(&iface->canard,
      data_type_signature,
      data_type_id, &iface->transfer_id,
      priority, payload, payload_len);
  chMtxUnlock(&iface->mutex);
  chEvtBroadcast(&iface->tx_request);
}
