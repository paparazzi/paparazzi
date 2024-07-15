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

#include "dronecan.h"

#ifndef UAVCAN_NODE_ID
#define UAVCAN_NODE_ID    100
#endif

#ifndef UAVCAN_BAUDRATE
#define UAVCAN_BAUDRATE   1000000
#endif

static dronecan_event *dronecan_event_hd = NULL;

#if UAVCAN_USE_CAN1
#ifndef UAVCAN_CAN1_NODE_ID
#define UAVCAN_CAN1_NODE_ID UAVCAN_NODE_ID
#endif

#ifndef UAVCAN_CAN1_BAUDRATE
#define UAVCAN_CAN1_BAUDRATE UAVCAN_BAUDRATE
#endif

static THD_WORKING_AREA(dronecan1_rx_wa, 1024 * 2);
static THD_WORKING_AREA(dronecan1_tx_wa, 1024 * 2);

struct dronecan_iface_t dronecan1 = {
  .can_driver = &CAND1,
  .can_baudrate = UAVCAN_CAN1_BAUDRATE,
  .can_cfg = {0},
  .thread_rx_wa = dronecan1_rx_wa,
  .thread_rx_wa_size = sizeof(dronecan1_rx_wa),
  .thread_tx_wa = dronecan1_tx_wa,
  .thread_tx_wa_size = sizeof(dronecan1_tx_wa),
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

static THD_WORKING_AREA(dronecan2_rx_wa, 1024 * 2);
static THD_WORKING_AREA(dronecan2_tx_wa, 1024 * 2);

struct dronecan_iface_t dronecan2 = {
  .can_driver = &CAND2,
  .can_baudrate = UAVCAN_CAN2_BAUDRATE,
  .can_cfg = {0},
  .thread_rx_wa = dronecan2_rx_wa,
  .thread_rx_wa_size = sizeof(dronecan2_rx_wa),
  .thread_tx_wa = dronecan2_tx_wa,
  .thread_tx_wa_size = sizeof(dronecan2_tx_wa),
  .node_id = UAVCAN_CAN2_NODE_ID,
  .transfer_id = 0,
  .initialized = false
};
#endif

/*
 * Receiver thread.
 */
static THD_FUNCTION(dronecan_rx, p)
{
  event_listener_t el;
  CANRxFrame rx_msg;
  CanardCANFrame rx_frame;
  struct dronecan_iface_t *iface = (struct dronecan_iface_t *)p;

  chRegSetThreadName("dronecan_rx");
  chEvtRegister(&iface->can_driver->rxfull_event, &el, EVENT_MASK(0));
  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    chMtxLock(&iface->mutex);

    // Wait until a CAN message is received
    while (canReceive(iface->can_driver, CAN_ANY_MAILBOX, &rx_msg, TIME_IMMEDIATE) == MSG_OK) {
      // Process message.
      const uint32_t timestamp = TIME_I2US(chVTGetSystemTimeX());
      memcpy(rx_frame.data, rx_msg.data8, 8);
      rx_frame.data_len = rx_msg.DLC;
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
      if (rx_msg.common.XTD) {
        rx_frame.id = CANARD_CAN_FRAME_EFF | rx_msg.ext.EID;
      } else {
        rx_frame.id = rx_msg.std.SID;
      }
#else
      if (rx_msg.IDE) {
        rx_frame.id = CANARD_CAN_FRAME_EFF | rx_msg.EID;
      } else {
        rx_frame.id = rx_msg.SID;
      }
#endif

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
static THD_FUNCTION(dronecan_tx, p)
{
  event_listener_t txc, txe, txr;
  struct dronecan_iface_t *iface = (struct dronecan_iface_t *)p;
  uint8_t err_cnt = 0;

  chRegSetThreadName("dronecan_tx");
  chEvtRegister(&iface->can_driver->txempty_event, &txc, EVENT_MASK(0));
  chEvtRegister(&iface->can_driver->error_event, &txe, EVENT_MASK(1));
  chEvtRegister(&iface->tx_request, &txr, EVENT_MASK(2));

  while (true) {
    eventmask_t evts = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));
    // Succesfull transmit
    if (evts == 0) {
      continue;
    }

    // Transmit error
    if (evts & EVENT_MASK(1)) {
      chEvtGetAndClearFlags(&txe);
      continue;
    }

    chMtxLock(&iface->mutex);
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&iface->canard)) != NULL;) {
      CANTxFrame tx_msg;
      memcpy(tx_msg.data8, txf->data, 8);
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
      tx_msg.DLC = txf->data_len; // TODO fixme for FDCAN (>8 bytes)
      tx_msg.ext.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
      tx_msg.common.XTD = 1;
      tx_msg.common.RTR = 0;
#else
      tx_msg.DLC = txf->data_len;
      tx_msg.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
      tx_msg.IDE = CAN_IDE_EXT;
      tx_msg.RTR = CAN_RTR_DATA;
#endif
      if (!canTryTransmitI(iface->can_driver, CAN_ANY_MAILBOX, &tx_msg)) {
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
  for (dronecan_event *ev = dronecan_event_hd; ev; ev = ev->next) {
    if (data_type_id == ev->data_type_id) {
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
  if (iface->can_baudrate < 1) {
    return false;
  }

  // Hardware configurationn
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  const uint32_t pclk = STM32_FDCANCLK;
#else
  const uint32_t pclk = STM32_PCLK1;
#endif
  static const int MaxBS1 = 16;
  static const int MaxBS2 = 8;

  /*
    * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
    *      CAN in Automation, 2003
    *
    * According to the source, optimal quanta per bit are:
    *   Bitrate        Optimal Maximum
    *   1000 kbps      8       10
    *   500  kbps      16      17
    *   250  kbps      16      17
    *   125  kbps      16      17
    */
  const int max_quanta_per_bit = (iface->can_baudrate >= 1000000) ? 10 : 17;
  static const int MaxSamplePointLocation = 900;

  /*
    * Computing (prescaler * BS):
    *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
    *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
    * let:
    *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
    *   PRESCALER_BS = PRESCALER * BS
    * ==>
    *   PRESCALER_BS = PCLK / BITRATE
    */
  const uint32_t prescaler_bs = pclk / iface->can_baudrate;

// Searching for such prescaler value so that the number of quanta per bit is highest.
  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;
  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
    if (bs1_bs2_sum <= 2) {
      return false;          // No solution
    }
    bs1_bs2_sum--;
  }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
  if ((prescaler < 1U) || (prescaler > 1024U)) {
    return false;              // No solution
  }

  /*
    * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
    * We need to find the values so that the sample point is as close as possible to the optimal value.
    *
    *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
    *   {{bs2 -> (1 + bs1)/7}}
    *
    * Hence:
    *   bs2 = (1 + bs1) / 7
    *   bs1 = (7 * bs1_bs2_sum - 1) / 8
    *
    * Sample point location can be computed as follows:
    *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
    *
    * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
    *   - With rounding to nearest
    *   - With rounding to zero
    */
// First attempt with rounding to nearest
  uint8_t bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8;
  uint8_t bs2 = bs1_bs2_sum - bs1;
  uint16_t sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);

// Second attempt with rounding to zero
  if (sample_point_permill > MaxSamplePointLocation) {
    bs1 = (7 * bs1_bs2_sum - 1) / 8;
    bs2 = bs1_bs2_sum - bs1;
    sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);
  }

  /*
    * Final validation
    * Helpful Python:
    * def sample_point_from_btr(x):
    *     assert 0b0011110010000000111111000000000 & x == 0
    *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
    *     return (1+ts1+1)/(1+ts1+1+ts2+1)
    *
    */
  if ((iface->can_baudrate != (pclk / (prescaler * (1 + bs1 + bs2)))) || (bs1 < 1) || (bs1 > MaxBS1) || (bs2 < 1)
      || (bs2 > MaxBS2)) {
    return false;
  }

  // Configure the interface
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  iface->can_cfg.NBTP = (0 << FDCAN_NBTP_NSJW_Pos) | ((bs1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((
                          bs2 - 1) << FDCAN_NBTP_NTSEG2_Pos) | ((prescaler - 1) << FDCAN_NBTP_NBRP_Pos);
  iface->can_cfg.CCCR = FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;
#else
  iface->can_cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  iface->can_cfg.btr = CAN_BTR_SJW(0) | CAN_BTR_TS1(bs1 - 1) | CAN_BTR_TS2(bs2 - 1) | CAN_BTR_BRP(prescaler - 1);
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
#if UAVCAN_USE_CAN1
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  // Configure the RAM
  dronecan1.can_cfg.RXF0C = (32 << FDCAN_RXF0C_F0S_Pos) | (0 << FDCAN_RXF0C_F0SA_Pos);
  dronecan1.can_cfg.RXF1C = (32 << FDCAN_RXF1C_F1S_Pos) | (128 << FDCAN_RXF1C_F1SA_Pos);
  dronecan1.can_cfg.TXBC  = (32 << FDCAN_TXBC_TFQS_Pos) | (256 << FDCAN_TXBC_TBSA_Pos);
  dronecan1.can_cfg.TXESC = 0x000; // 8 Byte mode only (4 words per message)
  dronecan1.can_cfg.RXESC = 0x000; // 8 Byte mode only (4 words per message)
#endif
  dronecanInitIface(&dronecan1);
#endif
#if UAVCAN_USE_CAN2
#if defined(STM32_CAN_USE_FDCAN1) || defined(STM32_CAN_USE_FDCAN2)
  // Configure the RAM
  dronecan2.can_cfg.RXF0C = (32 << FDCAN_RXF0C_F0S_Pos) | (384 << FDCAN_RXF0C_F0SA_Pos);
  dronecan2.can_cfg.RXF1C = (32 << FDCAN_RXF1C_F1S_Pos) | (512 << FDCAN_RXF1C_F1SA_Pos);
  dronecan2.can_cfg.TXBC  = (32 << FDCAN_TXBC_TFQS_Pos) | (640 << FDCAN_TXBC_TBSA_Pos);
  dronecan2.can_cfg.TXESC = 0x000; // 8 Byte mode only (4 words per message)
  dronecan2.can_cfg.RXESC = 0x000; // 8 Byte mode only (4 words per message)
#endif
  dronecanInitIface(&dronecan2);
#endif
}

/**
 * Bind to a receiving message from dronecan
 */
void dronecan_bind(uint16_t data_type_id, uint64_t data_type_signature, dronecan_event *ev, dronecan_callback cb)
{
  // Configure the event
  ev->data_type_id = data_type_id;
  ev->data_type_signature = data_type_signature;
  ev->cb = cb;
  ev->next = dronecan_event_hd;

  // Switch the head
  dronecan_event_hd = ev;
}

/**
 * Broadcast an dronecan message to a specific interface
 */
void dronecan_broadcast(struct dronecan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id,
                      uint8_t priority, const void *payload,
                      uint16_t payload_len)
{
  if (!iface->initialized) { return; }

  chMtxLock(&iface->mutex);
  canardBroadcast(&iface->canard,
                  data_type_signature,
                  data_type_id, &iface->transfer_id,
                  priority, payload, payload_len);
  chMtxUnlock(&iface->mutex);
  chEvtBroadcast(&iface->tx_request);
}
