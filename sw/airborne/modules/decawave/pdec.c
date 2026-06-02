/*
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com>
 *
 * This file is part of paparazzi. See LICENCE file.
 *
 * Pprz Decawave driver. Allows ranging, as well as TDOA.
 */

#include "modules/decawave/pdec.h"
#include "mcu_periph/uart.h"
#include "modules/datalink/downlink.h"
#include "modules/core/abi.h"
#include "modules/core/abi_sender_ids.h"

#include <stdio.h>
#include <string.h>

#define PDEC_SYNC_BYTE 0xff
#define PDEC_FRAME_OVERHEAD 4
#define PDEC_PACKED __attribute__((packed))

pdec_t pdec;

static void _pdec_init(pdec_t *pdec);
static void _pdec_periodic_report(pdec_t *pdec);
static void _pdec_event(pdec_t *pdec);

struct pdec_cmd_payload {
  uint8_t id;
} PDEC_PACKED;

struct pdec_target_cmd_payload {
  uint8_t id;
  uint16_t dst_id;
} PDEC_PACKED;

struct pdec_error_payload {
  uint8_t id;
  uint8_t command;
  uint8_t status;
} PDEC_PACKED;

struct pdec_distance_payload {
  uint8_t id;
  uint16_t dst_id;
  uint8_t status;
  float distance;
} PDEC_PACKED;

struct pdec_device_list_payload {
  uint8_t id;
  uint8_t count;
  uint16_t device_ids[];
} PDEC_PACKED;

struct pdec_send_data_result_payload {
  uint8_t id;
  uint16_t dst_id;
  uint8_t status;
  uint8_t sent_len;
} PDEC_PACKED;

struct pdec_ranging_event_payload {
  uint8_t id;
  uint8_t kind;
  uint16_t src_id;
  uint16_t dst_id;
  float distance;
} PDEC_PACKED;

struct pdec_tdoa_report_payload {
  uint16_t reporter_id;
  float distance;
} PDEC_PACKED;

struct pdec_tdoa_report_event_payload {
  uint8_t id;
  uint32_t blink_id;
  uint8_t expected_count;
  uint8_t count;
  uint8_t timed_out;
  struct pdec_tdoa_report_payload reports[];
} PDEC_PACKED;

static bool pdec_send_frame(pdec_t *pdec, const uint8_t *payload, uint8_t payload_len)
{
  uint8_t frame[PDEC_FRAME_OVERHEAD + PDEC_MAX_PAYLOAD_LEN];
  uint8_t checksum = payload_len;
  uint16_t frame_len = (uint16_t)payload_len + PDEC_FRAME_OVERHEAD;
  long fd = 0;

  if (payload_len == 0) {
    pdec->counters.tx_drops++;
    return false;
  }

  frame[0] = PDEC_SYNC_BYTE;
  frame[1] = PDEC_SYNC_BYTE;
  frame[2] = payload_len;
  for (uint8_t i = 0; i < payload_len; i++) {
    frame[3 + i] = payload[i];
    checksum ^= payload[i];
  }
  frame[3 + payload_len] = checksum;

  uart_put_buffer(pdec->dev, fd, frame, frame_len);
  pdec->counters.tx_frames++;
  return true;
}

static void pdec_parse_distance_result(struct pdec_distance_result *result,
                                       const struct pdec_distance_payload *payload)
{
  result->dst_id = payload->dst_id;
  result->status = (enum pdec_status)payload->status;
  result->distance = payload->distance;
  result->updated = true;
}

static void pdec_dispatch_frame(pdec_t *pdec)
{
  const uint8_t *payload = pdec->rx_payload;
  uint8_t len = pdec->rx_len;
  uint8_t msg_id;

  if (len == 0) {
    pdec->counters.rx_length_errors++;
    return;
  }

  msg_id = payload[0];
  switch (msg_id) {
    case PDEC_MSG_ERROR: {
      if (len != sizeof(struct pdec_error_payload)) {
        pdec->counters.rx_length_errors++;
        return;
      }
      const struct pdec_error_payload *msg = (const struct pdec_error_payload *)payload;
      pdec->last_error.command = msg->command;
      pdec->last_error.status = (enum pdec_status)msg->status;
      pdec->last_error.updated = true;
      break;
    }

    case PDEC_MSG_RANGE_RESULT:
      if (len != sizeof(struct pdec_distance_payload)) {
        pdec->counters.rx_length_errors++;
        return;
      }
      pdec_parse_distance_result(&pdec->last_range, (const struct pdec_distance_payload *)payload);
      break;

    case PDEC_MSG_DISTANCE:
      if (len != sizeof(struct pdec_distance_payload)) {
        pdec->counters.rx_length_errors++;
        return;
      }
      pdec_parse_distance_result(&pdec->last_distance, (const struct pdec_distance_payload *)payload);
      break;

    case PDEC_MSG_DEVICE_LIST: {
      if (len < sizeof(struct pdec_device_list_payload)) {
        pdec->counters.rx_length_errors++;
        return;
      }
      const struct pdec_device_list_payload *msg = (const struct pdec_device_list_payload *)payload;
      uint8_t total_count = msg->count;
      if (total_count > ((PDEC_MAX_PAYLOAD_LEN - sizeof(struct pdec_device_list_payload)) / sizeof(msg->device_ids[0])) ||
          len != (uint8_t)(sizeof(struct pdec_device_list_payload) + (total_count * sizeof(msg->device_ids[0])))) {
        pdec->counters.rx_length_errors++;
        return;
      }
      pdec->devices.total_count = total_count;
      pdec->devices.count = total_count > PDEC_MAX_DEVICES ? PDEC_MAX_DEVICES : total_count;
      pdec->devices.truncated = pdec->devices.total_count > pdec->devices.count;
      for (uint8_t i = 0; i < pdec->devices.count; i++) {
        pdec->devices.device_ids[i] = msg->device_ids[i];
      }
      pdec->devices.updated = true;
      break;
    }

    case PDEC_MSG_SEND_DATA_RESULT: {
      if (len != sizeof(struct pdec_send_data_result_payload)) {
        pdec->counters.rx_length_errors++;
        return;
      }
      const struct pdec_send_data_result_payload *msg = (const struct pdec_send_data_result_payload *)payload;
      pdec->last_send_data.dst_id = msg->dst_id;
      pdec->last_send_data.status = (enum pdec_status)msg->status;
      pdec->last_send_data.sent_len = msg->sent_len;
      pdec->last_send_data.updated = true;
      break;
    }

    case PDEC_MSG_RANGING_EVENT: {
      if (len != sizeof(struct pdec_ranging_event_payload)) {
        pdec->counters.rx_length_errors++;
        return;
      }
      const struct pdec_ranging_event_payload *msg = (const struct pdec_ranging_event_payload *)payload;
      pdec->last_ranging_event.kind = (enum pdec_event_kind)msg->kind;
      pdec->last_ranging_event.src_id = msg->src_id;
      pdec->last_ranging_event.dst_id = msg->dst_id;
      pdec->last_ranging_event.distance = msg->distance;
      pdec->last_ranging_event.updated = true;
      AbiSendMsgUWB_RANGING(UWB_PDEC_ID,
        get_sys_time_usec(),
        msg->src_id,
        msg->dst_id,
        msg->distance
      );
      break;
    }

    case PDEC_MSG_TDOA_REPORT_EVENT: {
      if (len < sizeof(struct pdec_tdoa_report_event_payload)) {
        pdec->counters.rx_length_errors++;
        return;
      }
      const struct pdec_tdoa_report_event_payload *msg = (const struct pdec_tdoa_report_event_payload *)payload;
      uint8_t total_count = msg->count;
      if (total_count > ((PDEC_MAX_PAYLOAD_LEN - sizeof(struct pdec_tdoa_report_event_payload)) /
                         sizeof(struct pdec_tdoa_report_payload)) ||
          len != (uint8_t)(sizeof(struct pdec_tdoa_report_event_payload) +
                           (total_count * sizeof(struct pdec_tdoa_report_payload)))) {
        pdec->counters.rx_length_errors++;
        return;
      }
      pdec->last_tdoa_report_event.blink_id = msg->blink_id;
      pdec->last_tdoa_report_event.expected_count = msg->expected_count;
      pdec->last_tdoa_report_event.total_count = total_count;
      pdec->last_tdoa_report_event.count = total_count > PDEC_MAX_TDOA_REPORTS ? PDEC_MAX_TDOA_REPORTS : total_count;
      pdec->last_tdoa_report_event.timed_out = msg->timed_out != 0;
      pdec->last_tdoa_report_event.truncated = pdec->last_tdoa_report_event.total_count > pdec->last_tdoa_report_event.count;
      uint16_t src_id[PDEC_MAX_TDOA_REPORTS];
      float range_diff[PDEC_MAX_TDOA_REPORTS];
      for (uint8_t i = 0; i < pdec->last_tdoa_report_event.count; i++) {
        src_id[i] = msg->reports[i].reporter_id;
        range_diff[i] = msg->reports[i].distance;
        pdec->last_tdoa_report_event.reports[i].reporter_id = src_id[i];
        pdec->last_tdoa_report_event.reports[i].distance = range_diff[i];
      }
      pdec->last_tdoa_report_event.updated = true;
      AbiSendMsgUWB_TDOA(UWB_PDEC_ID,
        get_sys_time_usec(),
        (uint16_t)msg->blink_id,
        pdec->last_tdoa_report_event.count,
        src_id,
        range_diff
      );
      break;
    }

    default:
      pdec->counters.rx_unknown_messages++;
      return;
  }

  pdec->counters.rx_frames++;
}

static void pdec_parse_byte(pdec_t *pdec, uint8_t byte)
{
  switch (pdec->rx_state) {
    case PDEC_RX_SYNC_1:
      if (byte == PDEC_SYNC_BYTE) {
        pdec->rx_state = PDEC_RX_SYNC_2;
      }
      break;

    case PDEC_RX_SYNC_2:
      if (byte == PDEC_SYNC_BYTE) {
        pdec->rx_state = PDEC_RX_LEN;
      } else {
        pdec->rx_state = PDEC_RX_SYNC_1;
      }
      break;

    case PDEC_RX_LEN:
      pdec->rx_len = byte;
      pdec->rx_idx = 0;
      pdec->rx_checksum = byte;
      pdec->rx_state = byte == 0 ? PDEC_RX_CHECKSUM : PDEC_RX_PAYLOAD;
      break;

    case PDEC_RX_PAYLOAD:
      pdec->rx_payload[pdec->rx_idx++] = byte;
      pdec->rx_checksum ^= byte;
      if (pdec->rx_idx >= pdec->rx_len) {
        pdec->rx_state = PDEC_RX_CHECKSUM;
      }
      break;

    case PDEC_RX_CHECKSUM:
      if (pdec->rx_checksum == byte) {
        pdec_dispatch_frame(pdec);
      } else {
        pdec->counters.rx_checksum_errors++;
      }
      pdec->rx_state = PDEC_RX_SYNC_1;
      break;

    default:
      pdec->rx_state = PDEC_RX_SYNC_1;
      break;
  }
}

static void _pdec_init(pdec_t *pdec)
{
  memset(pdec, 0, sizeof(*pdec));
  pdec->dev = &PDEC_UART_DEV;
  pdec->rx_state = PDEC_RX_SYNC_1;
}

static void _pdec_periodic_report(pdec_t *pdec __attribute__((unused)))
{
  char buf[100];

  if(pdec->last_error.updated) {
    
    
  }
  if(pdec->last_range.updated) {

    
    
  }
  if(pdec->last_distance.updated) {

  }
  if(pdec->devices.updated) {

  }
  if(pdec->last_send_data.updated) {

  }
  if(pdec->last_ranging_event.updated) {
    pdec->last_ranging_event.updated = false;

    int len = snprintf(buf, sizeof(buf), "[RNG]%x->%x:%0.2f",
      pdec->last_ranging_event.src_id,
      pdec->last_ranging_event.dst_id,
      pdec->last_ranging_event.distance);
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, len, buf);
  }
  if(pdec->last_tdoa_report_event.updated) {
    pdec->last_tdoa_report_event.updated = false;
    int offset = snprintf(buf, sizeof(buf), "[TDOA]");
    for(int i=0; i<pdec->last_tdoa_report_event.count; i++) {
      offset += snprintf(buf+offset, sizeof(buf)-offset, "%x:%0.2f,",
        pdec->last_tdoa_report_event.reports[i].reporter_id,
        pdec->last_tdoa_report_event.reports[i].distance);
    }
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, offset, buf);
  }

}

static void _pdec_event(pdec_t *pdec)
{
  while (uart_char_available(pdec->dev)) {
    pdec_parse_byte(pdec, uart_getch(pdec->dev));
  }
}

bool pdec_range(uint16_t dst_id)
{
  const struct pdec_target_cmd_payload payload = {
    .id = PDEC_CMD_RANGE,
    .dst_id = dst_id,
  };
  return pdec_send_frame(&pdec, (const uint8_t *)&payload, sizeof(payload));
}

bool pdec_get_distance(uint16_t dst_id)
{
  const struct pdec_target_cmd_payload payload = {
    .id = PDEC_CMD_GET_DISTANCE,
    .dst_id = dst_id,
  };
  return pdec_send_frame(&pdec, (const uint8_t *)&payload, sizeof(payload));
}

bool pdec_list_devices(void)
{
  const struct pdec_cmd_payload payload = {
    .id = PDEC_CMD_LIST_DEVICES,
  };
  return pdec_send_frame(&pdec, (const uint8_t *)&payload, sizeof(payload));
}

bool pdec_send_data(uint16_t dst_id, const uint8_t *data, uint8_t len)
{
  uint8_t payload[sizeof(struct pdec_target_cmd_payload) + PDEC_MAX_DATA_LEN];
  struct pdec_target_cmd_payload header = {
    .id = PDEC_CMD_SEND_DATA,
    .dst_id = dst_id,
  };

  if (len > PDEC_MAX_DATA_LEN || (len > 0 && data == NULL)) {
    pdec.counters.tx_drops++;
    return false;
  }

  memcpy(payload, &header, sizeof(header));
  if (len > 0) {
    memcpy(&payload[sizeof(header)], data, len);
  }

  return pdec_send_frame(&pdec, payload, (uint8_t)(sizeof(header) + len));
}



void pdec_init(void)
{
  _pdec_init(&pdec);
}

void pdec_periodic_report(void)
{
  _pdec_periodic_report(&pdec);
}

void pdec_event(void)
{
  _pdec_event(&pdec);
}


void uwb_range(uint16_t dst_id) {
  pdec_range(dst_id);
}
