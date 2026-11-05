/*
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com>
 *
 * This file is part of paparazzi. See LICENCE file.
 *
 * Pprz Decawave driver. Allows ranging, as well as TDOA.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

struct uart_periph;

#ifndef PDEC_MAX_DEVICES
#define PDEC_MAX_DEVICES 32
#endif

#ifndef PDEC_MAX_TDOA_REPORTS
#define PDEC_MAX_TDOA_REPORTS 32
#endif

#define PDEC_MAX_PAYLOAD_LEN 255
#define PDEC_MAX_DATA_LEN 252

enum pdec_command {
  PDEC_CMD_RANGE = 0x01,
  PDEC_CMD_GET_DISTANCE = 0x02,
  PDEC_CMD_LIST_DEVICES = 0x03,
  PDEC_CMD_SEND_DATA = 0x04,
};

enum pdec_msg {
  PDEC_MSG_ERROR = 0x7F,
  PDEC_MSG_RANGE_RESULT = 0x81,
  PDEC_MSG_DISTANCE = 0x82,
  PDEC_MSG_DEVICE_LIST = 0x83,
  PDEC_MSG_SEND_DATA_RESULT = 0x84,
  PDEC_MSG_RANGING_EVENT = 0x90,
  PDEC_MSG_TDOA_REPORT_EVENT = 0x92,
};

enum pdec_status {
  PDEC_STATUS_OK = 0x00,
  PDEC_STATUS_ERROR = 0x01,
  PDEC_STATUS_INVALID_LENGTH = 0x02,
  PDEC_STATUS_NOT_FOUND = 0x03,
  PDEC_STATUS_RANGE_FAILED = 0x04,
  PDEC_STATUS_MESSAGE_TOO_LONG = 0x05,
};

enum pdec_event_kind {
  PDEC_EVENT_SINGLE_SIDED = 0x00,
  PDEC_EVENT_DOUBLE_SIDED = 0x01,
  PDEC_EVENT_INDIRECT = 0x02,
};

enum pdec_rx_state {
  PDEC_RX_SYNC_1,
  PDEC_RX_SYNC_2,
  PDEC_RX_LEN,
  PDEC_RX_PAYLOAD,
  PDEC_RX_CHECKSUM,
};

struct pdec_error_response {
  bool updated;
  uint8_t command;
  enum pdec_status status;
};

struct pdec_distance_result {
  bool updated;
  uint16_t dst_id;
  enum pdec_status status;
  float distance;
};

struct pdec_device_list {
  bool updated;
  uint8_t count;
  uint8_t total_count;
  bool truncated;
  uint16_t device_ids[PDEC_MAX_DEVICES];
};

struct pdec_send_data_result {
  bool updated;
  uint16_t dst_id;
  enum pdec_status status;
  uint8_t sent_len;
};

struct pdec_ranging_event {
  bool updated;
  enum pdec_event_kind kind;
  uint16_t src_id;
  uint16_t dst_id;
  float distance;
};

struct pdec_tdoa_report {
  uint16_t reporter_id;
  float distance;
};

struct pdec_tdoa_report_event {
  bool updated;
  uint32_t blink_id;
  uint8_t expected_count;
  uint8_t count;
  uint8_t total_count;
  bool timed_out;
  bool truncated;
  struct pdec_tdoa_report reports[PDEC_MAX_TDOA_REPORTS];
};

struct pdec_counters {
  uint32_t rx_frames;
  uint32_t rx_checksum_errors;
  uint32_t rx_length_errors;
  uint32_t rx_unknown_messages;
  uint32_t tx_frames;
  uint32_t tx_drops;
};

typedef struct {
  struct uart_periph *dev;

  struct pdec_error_response last_error;
  struct pdec_distance_result last_range;
  struct pdec_distance_result last_distance;
  struct pdec_device_list devices;
  struct pdec_send_data_result last_send_data;
  struct pdec_ranging_event last_ranging_event;
  struct pdec_tdoa_report_event last_tdoa_report_event;
  struct pdec_counters counters;

  /* Driver-owned parser state. */
  enum pdec_rx_state rx_state;
  uint8_t rx_len;
  uint8_t rx_idx;
  uint8_t rx_checksum;
  uint8_t rx_payload[PDEC_MAX_PAYLOAD_LEN];
} pdec_t;

extern pdec_t pdec;

void pdec_init(void);
void pdec_periodic_report(void);
void pdec_event(void);

bool pdec_range(uint16_t dst_id);
bool pdec_get_distance(uint16_t dst_id);
bool pdec_list_devices(void);
bool pdec_send_data(uint16_t dst_id, const uint8_t *data, uint8_t len);
