#include "uavcan.h"
#include "mcu_periph/can.h"
#include "modules/core/threads.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo_req.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"
#include "uavcan_reporting.h"

#define AC_CAN_NAME_PREFIX "org.pprz."

static uavcan_event node_info_ev;

char ac_can_name[50] = {0};
uint8_t ac_can_name_len = 0;

static void node_info_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer);
static void get_uavcan_status(struct uavcan_protocol_NodeStatus* status);
static void get_uavcan_software_version (struct uavcan_protocol_SoftwareVersion* software_version);
static void get_uavcan_hardware_version(struct uavcan_protocol_HardwareVersion* hardware_version);


void uavcan_init_reporting() {
  strncpy(ac_can_name, AC_CAN_NAME_PREFIX, 50);
  size_t prefix_len = strlen(AC_CAN_NAME_PREFIX);
  strncpy(ac_can_name+prefix_len, AIRFRAME_NAME, 50-prefix_len);
  ac_can_name_len = prefix_len + strlen(AIRFRAME_NAME);

  uavcan_bind(UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID, UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE, &node_info_ev, node_info_cb);
}

void uavcan_reporting(void) {
  struct uavcan_protocol_NodeStatus report;
  get_uavcan_status(&report);

  uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
  uint32_t total_size = uavcan_protocol_NodeStatus_encode(&report, buffer);
  
#if UAVCAN_USE_CAN1
  uavcan_broadcast(&uavcan1, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID, CANARD_TRANSFER_PRIORITY_MEDIUM, buffer, total_size);
#endif
#if UAVCAN_USE_CAN2
  uavcan_broadcast(&uavcan2, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID, CANARD_TRANSFER_PRIORITY_MEDIUM, buffer, total_size);
#endif
}


static void get_uavcan_status(struct uavcan_protocol_NodeStatus* status) {
  status->uptime_sec = (uint32_t)get_sys_time_float();
  status->health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  status->mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  status->sub_mode = 0;
  status->vendor_specific_status_code = 0;
}

static void get_uavcan_software_version (struct uavcan_protocol_SoftwareVersion* software_version) {
  software_version->major = 6;
  software_version->minor = 0;
  software_version->optional_field_flags = 0;
  software_version->vcs_commit = 0;
  software_version->image_crc = 0;
}

static void get_uavcan_hardware_version(struct uavcan_protocol_HardwareVersion* hardware_version) {
  hardware_version->major = 2;
  hardware_version->minor = 1;
  memset(hardware_version->unique_id, 0, sizeof(hardware_version->unique_id));
  hardware_version->certificate_of_authenticity.len = 0;
}



static void node_info_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer __attribute__((unused))) {
  struct uavcan_protocol_GetNodeInfoResponse msg;
  get_uavcan_status(&msg.status);
  get_uavcan_software_version(&msg.software_version);
  get_uavcan_hardware_version(&msg.hardware_version);
  uint8_t len = Min(sizeof(msg.name.data), ac_can_name_len);
  strncpy((char*)msg.name.data, ac_can_name, len);
  msg.name.len = len;

  uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
  uint32_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&msg, buffer);

  CanardTxTransfer resp_transfer;
  canardInitTxTransfer(&resp_transfer);
  resp_transfer.data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE;
  resp_transfer.data_type_id = UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID;
  resp_transfer.inout_transfer_id = &transfer->transfer_id;
  resp_transfer.priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
  resp_transfer.payload = buffer;
  resp_transfer.payload_len = total_size;

  uavcan_response(iface, transfer->source_node_id, &resp_transfer);
}


