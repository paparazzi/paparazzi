 /*
 * Copyright (C) 2025 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

 /**
 *  Dynamic node ID allocation.
 *  See https://dronecan.github.io/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */

#include "uavcan/uavcan.h"
#include "uavcan/uavcan_allocator.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"
#include "uavcan.protocol.GetNodeInfo.h"

#ifndef UAVCAN_MAX_NODES
#define UAVCAN_MAX_NODES 50
#endif

#define INVALID_STAGE -1

static int detectRequestStage(struct uavcan_protocol_dynamic_node_id_Allocation* msg);
static int getExpectedStage(void);
static int findFreeNodeID(const uint8_t preferred);
static bool unique_id_identical(int index);
static void handleAllocationRequest(struct uavcan_iface_t *iface, uint8_t preferred_node_id);
static void id_alloc_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer);


static uavcan_event id_alloc_ev;
static uavcan_event node_info_ev;


// keep the correspondance between node id and unique IDs. (even or the fixed ids)
static struct uavcan_node_mapping_t uavcan_node_ids[UAVCAN_MAX_NODES] = {0};


struct uavcan_unique_id_t current_unique_id = {0};
uint32_t last_message_timestamp = 0;


struct uavcan_node_mapping_t* uavcan_get_node_id_mapping(const uint8_t id) {
  for(int i=0; i<UAVCAN_MAX_NODES; i++) {
    if(uavcan_node_ids[i].allocated_id == id) {
      return &uavcan_node_ids[i];
    }
  }
  return NULL;
}

static struct uavcan_node_mapping_t* get_free_id_mapping(void) {
  // a free slot is stored with node id == 0;
  return uavcan_get_node_id_mapping(0);
}

static int findFreeNodeID(const uint8_t preferred)
{
  // Search up
  int candidate = (preferred > 0) ? preferred : 125;
  while (candidate <= 125) {
      if (!uavcan_get_node_id_mapping(candidate)) {
        return candidate;
      }
      candidate++;
  }
  // Search down
  candidate = (preferred > 0) ? preferred : 125;
  while (candidate > 0) {
      if (!uavcan_get_node_id_mapping(candidate)) {
        return candidate;
      }
      candidate--;
  }
  // Not found
  return -1;
}


static int detectRequestStage(struct uavcan_protocol_dynamic_node_id_Allocation* msg) {
  if(msg->first_part_of_unique_id) {
    return 1;
  }
  if(msg->unique_id.len == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
    return 2;
  }
  if(msg->unique_id.len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
    return 3;
  }
  return INVALID_STAGE;  //invalid
}

static int getExpectedStage() {
  if(current_unique_id.len == 0) {
    return 1;
  }
  if(current_unique_id.len >= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST*2) {
    return 3;
  }
  if(current_unique_id.len >= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
    return 2;
  }
  return INVALID_STAGE;    //invalid
}

static bool unique_id_identical(int index) {
  return current_unique_id.len == uavcan_node_ids[index].unique_id.len &&
        memcmp(current_unique_id.data, uavcan_node_ids[index].unique_id.data, current_unique_id.len) == 0;
}

static void handleAllocationRequest(struct uavcan_iface_t *iface, uint8_t preferred_node_id) {
  uint8_t allocated_id = 0;

  for(int i=0; i<UAVCAN_MAX_NODES; i++) {

    if(uavcan_node_ids[i].allocated_id != 0) {
      // allocation slot taken, check if unique id matches
      if(unique_id_identical(i)) {
        allocated_id = uavcan_node_ids[i].allocated_id;
      }
    }
    else {
      // free allocation slot

      // copy unique id
      for(int k=0; k<current_unique_id.len; k++) {
        uavcan_node_ids[i].unique_id.data[k] = current_unique_id.data[k];
      }
      uavcan_node_ids[i].unique_id.len = current_unique_id.len;

      int free_id = findFreeNodeID(preferred_node_id);

      if(free_id != -1) {
        uavcan_node_ids[i].allocated_id = free_id;
        allocated_id = free_id;
        break;
      } else {
        // error: not more free IDs
        return;
      }
      
    }
  }

  struct uavcan_protocol_dynamic_node_id_Allocation response_msg;
  // copy unique id
  for(int k=0; k<current_unique_id.len; k++) {
    response_msg.unique_id.data[k] = current_unique_id.data[k];
  }
  response_msg.unique_id.len = current_unique_id.len;
  response_msg.node_id = allocated_id;

  uint8_t msg_buffer[50];

  uint32_t total_size = uavcan_protocol_dynamic_node_id_Allocation_encode(&response_msg, msg_buffer);

  uavcan_broadcast(
    iface,
    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
    CANARD_TRANSFER_PRIORITY_HIGH, msg_buffer, total_size);
}


static void id_alloc_uavcan_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer)
{
  struct uavcan_protocol_dynamic_node_id_Allocation msg;
  if(uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
    return;   // decode error
  }

  const int unique_id_capacity = sizeof(msg.unique_id.data);

  // only process anonymous transfers
  if(transfer->source_node_id != 0) {
    return;
  }

  uint32_t timestamp = transfer->timestamp_usec / 1000;

  // Reset the expected stage on timeout
  if(timestamp > last_message_timestamp + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS) {
    current_unique_id.len = 0;
  }

  // Checking if request stage matches the expected stage
  int request_stage = detectRequestStage(&msg);
  if(request_stage == INVALID_STAGE) { return; }
  if(request_stage != getExpectedStage()) { return; }
  if (msg.unique_id.len > (unique_id_capacity - current_unique_id.len)) { return; }

  // Updating the local state
  for(int i=0; i<msg.unique_id.len; i++) {
    current_unique_id.data[current_unique_id.len] = msg.unique_id.data[i];
    current_unique_id.len += 1;
  }

  if(current_unique_id.len == unique_id_capacity) {
    // Proceeding with allocation.
    handleAllocationRequest(iface, msg.node_id);
    current_unique_id.len = 0;
  } else {
    struct uavcan_protocol_dynamic_node_id_Allocation response_msg;
    // copy unique id
    for(int k=0; k<current_unique_id.len; k++) {
      response_msg.unique_id.data[k] = current_unique_id.data[k];
    }
    response_msg.unique_id.len = current_unique_id.len;

    uint8_t msg_buffer[50];

    uint32_t total_size = uavcan_protocol_dynamic_node_id_Allocation_encode(&response_msg, msg_buffer);

    uavcan_broadcast(
      iface,
      UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
      CANARD_TRANSFER_PRIORITY_HIGH, msg_buffer, total_size);
  }

  // It is important to update the timestamp only if the request has been processed successfully.
  last_message_timestamp = timestamp;
}

static void node_info_resp_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer) {
  (void)iface;
  struct uavcan_protocol_GetNodeInfoResponse msg;
  if(uavcan_protocol_GetNodeInfoResponse_decode(transfer, &msg)) {
    return;   // decode error
  }

  if(!uavcan_get_node_id_mapping(transfer->source_node_id)) {
    struct uavcan_node_mapping_t* mapping = get_free_id_mapping();
    mapping->allocated_id = transfer->source_node_id;
    for(int i=0; i<16; i++) {
      mapping->unique_id.data[i] = msg.hardware_version.unique_id[i];
    }
    mapping->unique_id.len = 16;
  }

}

void request_node_info(struct uavcan_iface_t *iface) {
  struct uavcan_protocol_GetNodeInfoRequest msg;
  uint8_t msg_buffer[10];
  uint32_t size = uavcan_protocol_GetNodeInfoRequest_encode(&msg, msg_buffer);
  uavcan_broadcast(
      iface,
      UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE, UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID,
      CANARD_TRANSFER_PRIORITY_HIGH, msg_buffer, size);
}


void uavcan_allocator_init(void) {
  uavcan_bind(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
    &id_alloc_ev, &id_alloc_uavcan_cb);
  
  uavcan_bind(UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE,
    &node_info_ev, &node_info_resp_cb);
}

