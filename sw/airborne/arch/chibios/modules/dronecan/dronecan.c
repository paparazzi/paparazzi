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
 * @file arch/chibios/modules/dronecan/dronecan.c
 * Interface from actuators to ChibiOS CAN driver using UAVCan
 *
 */

#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
#define FDCAN 1
#define CANARD_ENABLE_CANFD 1
#endif

#include "dronecan.h"
#include "canard_chutils.h"

#ifndef DRONECAN_NODE_ID
#define DRONECAN_NODE_ID        100
#endif

#ifndef DRONECAN_BAUDRATE
#define DRONECAN_BAUDRATE       1000000
#endif

#ifndef DRONECAN_BAUDRATE_MULT
#if FDCAN
#define DRONECAN_BAUDRATE_MULT  4
#else
#define DRONECAN_BAUDRATE_MULT  1
#endif
#endif

static dronecan_event *dronecan_event_hd = NULL;

#if DRONECAN_USE_CAN1
#ifndef DRONECAN_CAN1_NODE_ID
#define DRONECAN_CAN1_NODE_ID DRONECAN_NODE_ID
#endif

#ifndef DRONECAN_CAN1_BAUDRATE
#define DRONECAN_CAN1_BAUDRATE DRONECAN_BAUDRATE
#endif

#ifndef DRONECAN_CAN1_BAUDRATE_MULT
#define DRONECAN_CAN1_BAUDRATE_MULT DRONECAN_BAUDRATE_MULT
#endif

static THD_WORKING_AREA(dronecan1_rx_wa, 1024 * 2);
static THD_WORKING_AREA(dronecan1_tx_wa, 1024 * 2);

struct dronecan_iface_t dronecan1 = {
  .can_driver = &CAND1,
  .can_baudrate = DRONECAN_CAN1_BAUDRATE,
  .can_baudrate_mult = DRONECAN_CAN1_BAUDRATE_MULT,
  .fdcan_operation = FDCAN_ENABLE,
  .can_cfg = {0},
  .thread_rx_wa = dronecan1_rx_wa,
  .thread_rx_wa_size = sizeof(dronecan1_rx_wa),
  .thread_tx_wa = dronecan1_tx_wa,
  .thread_tx_wa_size = sizeof(dronecan1_tx_wa),
  .node_id = DRONECAN_CAN1_NODE_ID,
  .initialized = false
};
#endif

#if DRONECAN_USE_CAN2
#ifndef DRONECAN_CAN2_NODE_ID
#define DRONECAN_CAN2_NODE_ID DRONECAN_NODE_ID
#endif

#ifndef DRONECAN_CAN2_BAUDRATE
#define DRONECAN_CAN2_BAUDRATE DRONECAN_BAUDRATE
#endif

#ifndef DRONECAN_CAN2_BAUDRATE_MULT
#define DRONECAN_CAN2_BAUDRATE_MULT DRONECAN_BAUDRATE_MULT
#endif

static THD_WORKING_AREA(dronecan2_rx_wa, 1024 * 2);
static THD_WORKING_AREA(dronecan2_tx_wa, 1024 * 2);

struct dronecan_iface_t dronecan2 = {
  .can_driver = &CAND2,
  .can_baudrate = DRONECAN_CAN2_BAUDRATE,
  .can_baudrate_mult = DRONECAN_CAN1_BAUDRATE_MULT,
  .fdcan_operation = FDCAN_ENABLE,
  .can_cfg = {0},
  .thread_rx_wa = dronecan2_rx_wa,
  .thread_rx_wa_size = sizeof(dronecan2_rx_wa),
  .thread_tx_wa = dronecan2_tx_wa,
  .thread_tx_wa_size = sizeof(dronecan2_tx_wa),
  .node_id = DRONECAN_CAN2_NODE_ID,
  .initialized = false
};
#endif

/*
 * Receiver thread.
 */
static THD_FUNCTION(dronecan_rx, p)
{

  CANRxFrame rx_msg;
  CanardCANFrame rx_frame;
  struct dronecan_iface_t *iface = (struct dronecan_iface_t *)p;

  chRegSetThreadName("dronecan_rx");

  while (true) {
    // Wait until a CAN message is received
    while (canReceive(iface->can_driver, CAN_ANY_MAILBOX, &rx_msg, TIME_INFINITE) == MSG_OK) {
      // Process message.
      const uint64_t timestamp = TIME_I2US(chVTGetSystemTimeX());
      rx_frame = chibiRx2canard(rx_msg);
      // Let canard handle the frame
      chMtxLock(&iface->mutex);
      canardHandleRxFrame(&iface->canard, &rx_frame, timestamp);
      chMtxUnlock(&iface->mutex);
    }
  }
}

/*
 * Transmitter thread.
 */
static THD_FUNCTION(dronecan_tx, p)
{
  event_listener_t txemp, txerr, txreq;
  struct dronecan_iface_t *iface = (struct dronecan_iface_t *)p;
  uint8_t err_cnt = 0;

  chRegSetThreadName("dronecan_tx");
  chEvtRegister(&iface->can_driver->txempty_event, &txemp, EVENT_MASK(0));
  chEvtRegister(&iface->can_driver->error_event, &txerr, EVENT_MASK(1));
  chEvtRegister(&iface->tx_request, &txreq, EVENT_MASK(2));

  while (true) {
    eventmask_t evts = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));
    // Succesfull transmit
    if (evts == 0) {
      continue;
    }

    // Transmit error
    if (evts & EVENT_MASK(1)) {
      chEvtGetAndClearFlags(&txerr);
      continue;
    }

    chMtxLock(&iface->mutex);
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&iface->canard)) != NULL;) {
      CANTxFrame tx_msg;
      tx_msg = canard2chibiTx(txf);
      if (!canTryTransmitI(iface->can_driver, CAN_ANY_MAILBOX, &tx_msg)) {
        err_cnt = 0;
        canardPopTxQueue(&iface->canard);
      } else {
        // After 5 retries giveup and clean full queue
        if (err_cnt >= 5) {
          err_cnt = 0;
          while (canardPeekTxQueue(&iface->canard)) { 
            canardPopTxQueue(&iface->canard); 
          }
          continue;
        }

        // Timeout - just exit and try again later
        chMtxUnlock(&iface->mutex);
        chThdSleepMilliseconds(++err_cnt);
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
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
  struct dronecan_iface_t *iface = (struct dronecan_iface_t *)ins->user_reference;

  // Go through all registered callbacks and call function callback if found
  for (dronecan_event *ev = dronecan_event_hd; ev; ev = ev->next) {
    if ((transfer->transfer_type == ev->transfer_type)&&(transfer->data_type_id == ev->data_type_id)) {
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
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id __attribute__((unused)))
{

  // Go through all registered callbacks and return signature if found
  for (dronecan_event *ev = dronecan_event_hd; ev; ev = ev->next) {
    if ((transfer_type == ev->transfer_type)&&(data_type_id == ev->data_type_id)) {
      *out_data_type_signature = ev->data_type_signature;
      return true;
    }
  }
  // No callback found return
  return false;
}

/**
 * Try to compute the timing registers for the can interface and set the configuration
 */
static bool dronecanConfigureIface(struct dronecan_iface_t *iface)
{
  if (!((iface->can_baudrate == 1000000) ||
        (iface->can_baudrate == 500000)  ||
        (iface->can_baudrate == 250000))) {
    return false;
  }

  if (!((iface->can_baudrate_mult == 1) ||
        (iface->can_baudrate_mult == 2) ||
        (iface->can_baudrate_mult == 4))) {
    return false;
  } 

  // Hardware configurationn
#if FDCAN
  const uint32_t pclk = STM32_FDCANCLK;
#else
  const uint32_t pclk = STM32_PCLK1;
#endif

  uint32_t nb_tq = pclk/(iface->can_baudrate*iface->can_baudrate_mult);

  uint32_t t_seg1 = (nb_tq*0.875) - 1;
  uint32_t t_seg2 =  nb_tq - t_seg1;

  // Configure the interface
#if FDCAN
  iface->can_cfg.op_mode = OPMODE_FDCAN;
  iface->can_cfg.NBTP = FDCAN_CONFIG_NBTP_NSJW(0U) |
                        FDCAN_CONFIG_NBTP_NBRP(iface->can_baudrate_mult - 1) | 
                        FDCAN_CONFIG_NBTP_NTSEG1(t_seg1 - 1) |
                        FDCAN_CONFIG_NBTP_NTSEG2(t_seg2 - 1);  

  iface->can_cfg.DBTP = FDCAN_CONFIG_DBTP_DSJW(3U) |
                        FDCAN_CONFIG_DBTP_DBRP(0U) |
                        FDCAN_CONFIG_DBTP_DTSEG1(t_seg1 - 1) |
                        FDCAN_CONFIG_DBTP_DTSEG2(t_seg2 - 1) |
                        FDCAN_CONFIG_DBTP_TDC(0U);
  iface->can_cfg.CCCR = FDCAN_CCCR_BRSE;
  iface->can_cfg.RXGFC = FDCAN_CONFIG_GFC_ANFE_REJECT | // Réjection des trames étendues non
                                                        // acceptées.
                         FDCAN_CONFIG_GFC_ANFS_REJECT | // Réjection des trames standard non
                                                        // acceptées.
                         // Pas de trames distantes avec DroneCAN.
                         FDCAN_CONFIG_GFC_RRFE | // Réjection des trames distantes étendues.
                         FDCAN_CONFIG_GFC_RRFS;  // Réjection des trames distantes standard.
#else
  iface->can_cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  iface->can_cfg.btr = CAN_BTR_SJW(0) |
                       CAN_BTR_TS1(t_seg1 - 1) |
                       CAN_BTR_TS2(t_seg2 - 1) |
                       CAN_BTR_BRP(0U);
#endif
  return true;
}

/**
 * Initialize dronecan interface
 */
static void dronecanInitIface(struct dronecan_iface_t *iface)
{
  // First try to configure abort if failed
  if (!dronecanConfigureIface(iface)) {
    return;
  }

  // Initialize mutexes/events for multithread locking
  chMtxObjectInit(&iface->mutex);
  chEvtObjectInit(&iface->tx_request);

  // Initialize canard
  canardInit(&iface->canard, iface->canard_memory_pool, sizeof(iface->canard_memory_pool),
             onTransferReceived, shouldAcceptTransfer, iface);
  // Update the dronecan node ID
  canardSetLocalNodeID(&iface->canard, iface->node_id);

  // Start the can interface
  canStart(iface->can_driver, &iface->can_cfg);

  // Start the receiver and transmitter thread
  chThdCreateStatic(iface->thread_rx_wa, iface->thread_rx_wa_size, NORMALPRIO + 8, dronecan_rx, (void *)iface);
  chThdCreateStatic(iface->thread_tx_wa, iface->thread_tx_wa_size, NORMALPRIO + 7, dronecan_tx, (void *)iface);
  iface->initialized = true;
}

/**
 * Initialize all dronecan interfaces
 */
void dronecan_init(void)
{
#if DRONECAN_USE_CAN1
  dronecanInitIface(&dronecan1);
#endif
#if DRONECAN_USE_CAN2
  dronecanInitIface(&dronecan2);
#endif
}

/**
 * Bind to a receiving message from dronecan
 */
void dronecan_bind(CanardTransferType transfer_type, uint16_t data_type_id, uint64_t data_type_signature, dronecan_event *ev, dronecan_callback cb)
{
  // Configure the event
  ev->transfer_type = transfer_type;
  ev->data_type_id = data_type_id;
  ev->data_type_signature = data_type_signature;
  ev->cb = cb;
  ev->next = dronecan_event_hd;

  // Switch the head
  dronecan_event_hd = ev;
}

/**
 * Broadcast a dronecan message on a specific interface
 */
void dronecan_broadcast(struct dronecan_iface_t *iface, CanardTxTransfer *transfer)
{
  if (!iface->initialized) { return; }

  chMtxLock(&iface->mutex);
  canardBroadcastObj(&iface->canard, transfer);
  chMtxUnlock(&iface->mutex);
  chEvtBroadcast(&iface->tx_request);
}

/**
 * Send a dronecan request/response on a specific interface
 */
void dronecan_request_or_respond(struct dronecan_iface_t *iface, uint8_t destination_node_id,CanardTxTransfer *transfer)
{
  if (!iface->initialized) { return; }

  chMtxLock(&iface->mutex);
  canardRequestOrRespondObj(&iface->canard, destination_node_id, transfer);
  chMtxUnlock(&iface->mutex);
  chEvtBroadcast(&iface->tx_request);
}