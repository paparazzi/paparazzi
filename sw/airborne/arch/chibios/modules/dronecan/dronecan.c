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
 * Interface from actuators to ChibiOS CAN driver using DroneCAN
 *
 */

#include "dronecan.h"

#ifndef DRONECAN_NODE_ID
#define DRONECAN_NODE_ID        42
#endif

#ifndef DRONECAN_BAUDRATE
#define DRONECAN_BAUDRATE       1000000
#endif

#ifndef MAX_CAN_NODES
#define MAX_CAN_NODES           128
#endif

#ifndef VENDOR_STATUS
#define VENDOR_STATUS           0xA5A5
#endif

#ifndef FDCAN_ENABLED
#define FDCAN_ENABLED           TRUE
#endif

#ifndef DRONECAN_BAUDRATE_MULT
#if FDCAN_ENABLED
#define DRONECAN_BAUDRATE_MULT  4
#else
#define DRONECAN_BAUDRATE_MULT  1
#endif
#endif

static dronecan_event *dronecan_event_hd = NULL;

#if DRONECAN_USE_CAN1
#define FDCAN_PERIPH 1

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
static THD_WORKING_AREA(dronecan1_tx_wa, 1024);
static THD_WORKING_AREA(dronecan1_ns_wa, 1024);

struct dronecan_iface_t dronecan1 = {
  .can_driver = &CAND1,
  .can_baudrate = DRONECAN_CAN1_BAUDRATE,
  .can_baudrate_mult = DRONECAN_CAN1_BAUDRATE_MULT,
  .fdcan_operation = FDCAN_ENABLED,
  .can_cfg = {0},
  .thread_rx_wa = dronecan1_rx_wa,
  .thread_rx_wa_size = sizeof(dronecan1_rx_wa),
  .thread_tx_wa = dronecan1_tx_wa,
  .thread_tx_wa_size = sizeof(dronecan1_tx_wa),
  .thread_ns_wa = dronecan1_ns_wa,
  .thread_ns_wa_size = sizeof(dronecan1_ns_wa),
  .node_id = DRONECAN_CAN1_NODE_ID,
  .initialized = false
};
#endif

#if DRONECAN_USE_CAN2
#define FDCAN_PERIPH 1

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
static THD_WORKING_AREA(dronecan2_tx_wa, 1024);
static THD_WORKING_AREA(dronecan2_ns_wa, 1024);

struct dronecan_iface_t dronecan2 = {
  .can_driver = &CAND2,
  .can_baudrate = DRONECAN_CAN2_BAUDRATE,
  .can_baudrate_mult = DRONECAN_CAN2_BAUDRATE_MULT,
  .fdcan_operation = FDCAN_ENABLED,
  .can_cfg = {0},
  .thread_rx_wa = dronecan2_rx_wa,
  .thread_rx_wa_size = sizeof(dronecan2_rx_wa),
  .thread_tx_wa = dronecan2_tx_wa,
  .thread_tx_wa_size = sizeof(dronecan2_tx_wa),
  .thread_ns_wa = dronecan2_ns_wa,
  .thread_ns_wa_size = sizeof(dronecan2_ns_wa),
  .node_id = DRONECAN_CAN2_NODE_ID,
  .initialized = false
};
#endif

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

  canardBroadcastObj(&iface->canard, transfer);
  chEvtBroadcast(&iface->tx_request);
}

/**
 * Send a dronecan request/response on a specific interface
 */
void dronecan_request_or_respond(struct dronecan_iface_t *iface, uint8_t destination_node_id,CanardTxTransfer *transfer)
{
  if (!iface->initialized) { return; }

  canardRequestOrRespondObj(&iface->canard, destination_node_id, transfer);
  chEvtBroadcast(&iface->tx_request);
}

static void initNodesList(struct dronecan_iface_t *iface) {
  for (uint8_t i = 0; i < MAX_CAN_NODES; i++) {
    iface->nodes_list[i].active = 0;
    iface->nodes_list[i].timestamp_usec = 0;
  };
}

static void getNodeInfo(struct dronecan_iface_t *iface, uint8_t dest_node_id) {

  static uint8_t transfer_id = 0;
  static CanardTxTransfer request;
  canardInitTxTransfer(&request);

  request.transfer_type = CanardTransferTypeRequest;
  request.data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
  request.data_type_id = UAVCAN_PROTOCOL_GETNODEINFO_ID;
  request.inout_transfer_id = &transfer_id;
  request.priority = CANARD_TRANSFER_PRIORITY_LOW;
  request.payload = NULL; // No data sent by this request.
  request.payload_len = 0;
  request.canfd = iface->fdcan_operation;
  request.tao = !((iface->canard.tao_disabled) || request.canfd);

  dronecan_request_or_respond(iface, dest_node_id, &request);
}

static void updateNodesList(struct dronecan_iface_t *iface, uint8_t node_id,
                            uint64_t timestamp) {
  chDbgAssert(node_id < MAX_CAN_NODES, "erreur sur nodeID");
  if (iface->nodes_list[node_id].active == 0) {
    getNodeInfo(iface, node_id);
  } else {
    iface->nodes_list[node_id].timestamp_usec = timestamp;
  };
}

static void cb_NodeStatus(struct dronecan_iface_t *iface, CanardRxTransfer *transfer) {
  struct uavcan_protocol_NodeStatus status;
  bool decode_error;
  const uint8_t node_id = transfer->source_node_id;
  const uint64_t timestamp = transfer->timestamp_usec;

  decode_error = uavcan_protocol_NodeStatus_decode(transfer, &status);
  if (!decode_error) {
    updateNodesList(iface, node_id, timestamp);
  }
}

static void cb_GetNodeInfoRequest(struct dronecan_iface_t *iface, CanardRxTransfer *transfer) {
  uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
  struct uavcan_protocol_GetNodeInfoResponse pkt = {};

  iface->node_status.uptime_sec = (uint32_t)chTimeI2S(chVTGetSystemTime());
  pkt.status = iface->node_status;

  // paparazzi version.
  pkt.software_version.major = PPRZ_VER_MAJOR;
  pkt.software_version.minor = PPRZ_VER_MINOR;
  pkt.software_version.optional_field_flags = 0;
  pkt.software_version.vcs_commit = 0; // git hash

  // board version.
  pkt.hardware_version.major = 2;
  pkt.hardware_version.minor = 1;

  getUniqueID((UniqId_t *)pkt.hardware_version.unique_id);

  strncpy((char *)pkt.name.data, "TawakiNode", sizeof(pkt.name.data));
  pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));

  uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(
      &pkt, buffer, !((iface->canard.tao_disabled) || (transfer->canfd)));

  static CanardTxTransfer response;
  canardInitTxTransfer(&response);

  response.transfer_type = CanardTransferTypeResponse;
  response.data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
  response.data_type_id = UAVCAN_PROTOCOL_GETNODEINFO_ID;
  response.inout_transfer_id = &transfer->transfer_id;
  response.priority = transfer->priority;
  response.payload = buffer;
  response.payload_len = total_size;
  response.canfd = iface->fdcan_operation;
  response.tao = !((iface->canard.tao_disabled) || (response.canfd));

  dronecan_request_or_respond(iface,transfer->source_node_id,&response);
}

static void cb_GetNodeInfoResponse(struct dronecan_iface_t *iface, CanardRxTransfer *transfer) {
  struct uavcan_protocol_GetNodeInfoResponse res;
  bool decode_error;
  const uint8_t node_id = transfer->source_node_id;
  const uint64_t timestamp = transfer->timestamp_usec;

  decode_error = uavcan_protocol_GetNodeInfoResponse_decode(transfer, &res);
  if (!decode_error) {
    chDbgAssert(node_id < MAX_CAN_NODES, "erreur sur nodeID");
    iface->nodes_list[node_id].active = 1;
    iface->nodes_list[node_id].timestamp_usec = timestamp;
  }
}

static void sendNodeStatus(struct dronecan_iface_t *iface) {
  uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];

  iface->node_status.uptime_sec = (uint32_t)chTimeI2S(chVTGetSystemTime());
  iface->node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  iface->node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  iface->node_status.sub_mode = 0;
  iface->node_status.vendor_specific_status_code = VENDOR_STATUS;

  uint32_t len = uavcan_protocol_NodeStatus_encode(
      &iface->node_status, buffer, !((iface->canard.tao_disabled) || iface->fdcan_operation));

  static uint8_t transfer_id = 0;
  static CanardTxTransfer broadcast;
  canardInitTxTransfer(&broadcast);

  broadcast.transfer_type = CanardTransferTypeBroadcast;
  broadcast.data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
  broadcast.data_type_id = UAVCAN_PROTOCOL_NODESTATUS_ID;
  broadcast.inout_transfer_id = &transfer_id;
  broadcast.priority = CANARD_TRANSFER_PRIORITY_LOW;
  broadcast.payload = buffer;
  broadcast.payload_len = len;
  broadcast.canfd = iface->fdcan_operation;
  broadcast.tao = !((iface->canard.tao_disabled) || broadcast.canfd);

  dronecan_broadcast(iface, &broadcast);
}

/*
 * NodeStatus generator thread.
 */
static THD_FUNCTION(dronecan_ns, p) 
{
  struct dronecan_iface_t *iface = (struct dronecan_iface_t *)p;
  chRegSetThreadName("dronecan_ns");
  static uint32_t timestamp, current_time_ms;
  while (true) {
    for (uint8_t i = 0; i < MAX_CAN_NODES; i++) {
      if (iface->nodes_list[i].active) {
        timestamp = iface->nodes_list[i].timestamp_usec / (1000UL);
        current_time_ms = chTimeI2MS(chVTGetSystemTime());
        if (current_time_ms > timestamp) {
          iface->nodes_list[i].active =
              (current_time_ms - timestamp <
               UAVCAN_PROTOCOL_NODESTATUS_OFFLINE_TIMEOUT_MS);
        }
      }
    }

    chMtxLock(&iface->mutex);
    canardCleanupStaleTransfers(&iface->canard,
                                (uint64_t)chTimeI2US(chVTGetSystemTime()));
    sendNodeStatus(iface);
    chMtxUnlock(&iface->mutex);

    chThdSleepMilliseconds(1000);
  }
}

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
    while (canReceiveTimeout(iface->can_driver, CAN_ANY_MAILBOX, &rx_msg, TIME_INFINITE) == MSG_OK) {
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
  event_listener_t txreq;
  struct dronecan_iface_t *iface = (struct dronecan_iface_t *)p;
  uint8_t err_cnt = 0;

  chRegSetThreadName("dronecan_tx");
  chEvtRegister(&iface->tx_request, &txreq, 0);

  while (true) {
    chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));
    chMtxLock(&iface->mutex);
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&iface->canard)) != NULL;) {
      CANTxFrame tx_msg;
      tx_msg = canard2chibiTx(txf);
      msg_t tx_ok =
        canTransmitTimeout(iface->can_driver, CAN_ANY_MAILBOX, &tx_msg, chTimeMS2I(100));
      switch (tx_ok) {
        case MSG_OK: {
          err_cnt = 0;
          canardPopTxQueue(&iface->canard);
        break;
        } 
        case MSG_TIMEOUT: {
          err_cnt++;
        break;
        }
        case MSG_RESET: {
          err_cnt++;
          canardPopTxQueue(&iface->canard);
        break;
        }
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
#if FDCAN_PERIPH
  const uint32_t pclk = STM32_FDCANCLK;
#else
  const uint32_t pclk = STM32_PCLK1;
#endif

  uint32_t nb_tq = pclk/(iface->can_baudrate*iface->can_baudrate_mult);

  uint32_t t_seg1 = (uint32_t)(nb_tq*0.875) - 1;
  uint32_t t_seg2 =  nb_tq - t_seg1 - 1;

  // Configure the interface
#if FDCAN_PERIPH
  iface->can_cfg.op_mode = OPMODE_FDCAN;
  iface->can_cfg.NBTP = FDCAN_CONFIG_NBTP_NSJW(0U) |
                        FDCAN_CONFIG_NBTP_NBRP(iface->can_baudrate_mult - 1) | 
                        FDCAN_CONFIG_NBTP_NTSEG1(t_seg1 - 1) |
                        FDCAN_CONFIG_NBTP_NTSEG2(t_seg2 - 1);  

  iface->can_cfg.DBTP = FDCAN_CONFIG_DBTP_DSJW(3U) |
                        FDCAN_CONFIG_DBTP_DBRP(0U) |
                        FDCAN_CONFIG_DBTP_DTSEG1(t_seg1 - 1) |
                        FDCAN_CONFIG_DBTP_DTSEG2(t_seg2 - 1);
                        // FDCAN_CONFIG_DBTP_TDC = 0
  iface->can_cfg.CCCR = FDCAN_CCCR_BRSE;
  iface->can_cfg.RXGFC = //FDCAN_CONFIG_GFC_ANFE_REJECT | // Réjection des trames étendues non
                                                        // acceptées.
                         FDCAN_CONFIG_GFC_ANFS_REJECT | // Réjection des trames standard non
                                                        // acceptées.
                         // Pas de trames distantes avec DroneCAN.
                         FDCAN_CONFIG_GFC_RRFE | // Réjection des trames distantes étendues.
                         FDCAN_CONFIG_GFC_RRFS;  // Réjection des trames distantes standard.
#else
  iface->can_cfg.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM;
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

  initNodesList(iface);

  // Initialize mutexes/events for multithread locking
  chMtxObjectInit(&iface->mutex);
  chEvtObjectInit(&iface->tx_request);

  // Initialize canard
  canardInit(&iface->canard, iface->canard_memory_pool, sizeof(iface->canard_memory_pool),
             onTransferReceived, shouldAcceptTransfer, iface);
  // Update the dronecan node ID
  canardSetLocalNodeID(&iface->canard, iface->node_id);

  static dronecan_event rec_NodeStatus, rec_GetNodeInfoRequest, rec_GetNodeInfoResponse;
  dronecan_bind(CanardTransferTypeBroadcast,UAVCAN_PROTOCOL_NODESTATUS_ID,
                UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,&rec_NodeStatus,&cb_NodeStatus);
  dronecan_bind(CanardTransferTypeRequest,UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID,
                UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE,&rec_GetNodeInfoRequest,&cb_GetNodeInfoRequest);
  dronecan_bind(CanardTransferTypeResponse,UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID,
                UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE,&rec_GetNodeInfoResponse,&cb_GetNodeInfoResponse);

  // Start the can interface
  canStart(iface->can_driver, &iface->can_cfg);

  iface->initialized = true;

  // Start the receiver and transmitter thread
  chThdCreateStatic(iface->thread_tx_wa, iface->thread_tx_wa_size, NORMALPRIO + 7, dronecan_tx, (void *)iface);
  chThdCreateStatic(iface->thread_ns_wa, iface->thread_ns_wa_size, NORMALPRIO + 9, dronecan_ns, (void *)iface);
  chThdCreateStatic(iface->thread_rx_wa, iface->thread_rx_wa_size, NORMALPRIO + 8, dronecan_rx, (void *)iface);
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