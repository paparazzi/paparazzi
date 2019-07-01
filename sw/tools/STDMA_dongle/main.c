//
// This is an adaptation of Bluegiga's Bluetooth Smart Demo Application to support STDMA 
// communication between different dongles. This way, drones with the module can communicate 
// and measure their RSSI.
//
// Contact: support@bluegiga.com.
//
// This is free software distributed under the terms of the MIT license reproduced below.
//
// Copyright (c) 2012, Bluegiga Technologies
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>    // time()
#include <signal.h>

#include "cmd_def.h"
#include "uart.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#ifdef PRINT
#define debug_print(...) printf(__VA_ARGS__)
#else
#define debug_print(...)
#endif

int sock;
int bytes_read;
char recv_data[1024], send_data[1024];
struct sockaddr_in server_addr, client_addr;

#define DEBUG
#define UART_TIMEOUT 0//100

uint8 MAC_ADDR[] = {0x0, 0x00, 0x1e, 0x80, 0x07, 0x00}; //{0x40,0xe0,0x2d,0x80,0x07,0x00};

FILE *fp;

// ADVertizement data
uint8_t adv_data[31];
uint8_t adv_data_len;

// STDMA variables
const uint8_t STDMA_ADV_DATA_HEADER_LEN = 6;      // length of header for custom data
const uint8_t STDMA_ADV_HEADER_LEN = 10;          // total length of response data package
const uint8_t STDMA_ADV_MAX_DATA_LEN = 21;        // Max data length
const uint8_t STDMA_SLOTS = 8;
// note: transmit packets per frame is fixed to 1

const uint8_t STDMA_MIN_INTERVAL = 3;           // Min nr of frames before required to reselect
const uint8_t STDMA_MAX_INTERVAL = 7;           // Max nr of frames before required to reselect
const uint8_t STDMA_SELECTION_INTERVAL = 2;     // Number of slots to consider around the nominal increment
const uint32_t STDMA_FRAME_DURATION = 6553;       // frame interval [clock ticks, 32768 = 1s]

// STDMA states, listed in decreasing priority: internally allocated, externally allocated, busy, free
const uint8_t STDMA_STATE_INTER_ALLOC = 3;
const uint8_t STDMA_STATE_EXTER_ALLOC = 2;
const uint8_t STDMA_STATE_BUSY = 1;
const uint8_t STDMA_STATE_FREE = 0;

uint8_t stdma_slot_status[8];               // status of each slot stored here
uint8_t stdma_slot_timeout[8];              // time a reservation is valid
uint8_t stdma_next_slot_timeout[8];         // next frame reservation
uint8_t stdma_free_slots[8];                // temporary store of all free slots

uint8_t stdma_my_slot;
uint8_t stdma_my_next_slot;
uint8_t stdma_current_slot;
uint8_t stdma_braodcasting;

// advertisement buffer positions
const uint8_t POS_ADV_LEN = 3;                        // length of advertisement data
const uint8_t POS_ADV_STDMA_OFFSET = 7;               // stdma offset
const uint8_t POS_ADV_STDMA_TIMEOUT = 8;              // stdma timeout
const uint8_t POS_ADV_TX_STRENGTH = 9;                // transmit power dBm

uint8_t broadcast_msg[128];                    // temporary buffer for recieved data waiting to be sent to lisa
int8_t broadcast_len;

// PPRZ MSG positions
const uint8_t PPRZ_POS_STX = 0;
const uint8_t PPRZ_POS_LEN = 1;
const uint8_t PPRZ_POS_SENDER_ID = 2;
const uint8_t PPRZ_POS_MSG_ID = 3;

/**
 * Compare Bluetooth addresses
 *
 * @param first First address
 * @param second Second address
 * @return Zero if addresses are equal
 */
int cmp_bdaddr(const bd_addr first, const bd_addr second)
{
  int i;
  for (i = 0; i < sizeof(bd_addr); i++) {
    if (first.addr[i] != second.addr[i]) { return 1; }
  }
  return 0;
}

int cmp_addr(const uint8 first[], const uint8 second[])
{
  int i;
  for (i = 5; i >= 0; i--) {
    if (first[i] != second[i]) { return 5 - i; }
  }
  return 6;
}

void print_bdaddr(const bd_addr bdaddr)
{
  debug_print("%02x:%02x:%02x:%02x:%02x:%02x",
              bdaddr.addr[5],
              bdaddr.addr[4],
              bdaddr.addr[3],
              bdaddr.addr[2],
              bdaddr.addr[1],
              bdaddr.addr[0]);
}

void print_raw_packet(struct ble_header *hdr, unsigned char *data)
{
  debug_print("Incoming packet (len: %d): ", hdr->lolen);
  int i;
  for (i = 0; i < sizeof(*hdr); i++) {
    debug_print("%02x ", ((unsigned char *)hdr)[i]);
  }
  for (i = 0; i < hdr->lolen; i++) {
    debug_print("%02x ", data[i]);
  }
  debug_print("\n");
}

void output(uint8 len1, uint8 *data1, uint16 len2, uint8 *data2)
{
  if (uart_tx(len1, data1) || uart_tx(len2, data2)) {
    debug_print("ERROR: Writing to serial port failed\n");
    //exit(1);
  }
}

int read_message(int timeout_ms)
{
  unsigned char data[256]; // enough for BLE
  struct ble_header hdr;
  int r;

  r = uart_rx(sizeof(hdr), (unsigned char *)&hdr, UART_TIMEOUT);
  if (!r) {
    return -1; // timeout
  } else if (r < 0) {
    debug_print("ERROR: Reading header failed. Error code:%d\n", r);
    return 1;
  }

  if (hdr.lolen) {
    r = uart_rx(hdr.lolen, data, UART_TIMEOUT);
    if (r <= 0) {
      debug_print("ERROR: Reading data failed. Error code:%d\n", r);
      return 1;
    }
  }

  const struct ble_msg *msg = ble_get_msg_hdr(hdr);

#ifdef DEBUG_RAW
  print_raw_packet(&hdr, data);
#endif

  if (!msg) {
    debug_print("ERROR: Unknown message received\n");
    exit(1);
  }

  msg->handler(data);
  return 0;
}

void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{
  // check if sender is likely a bluegiga module or dongle
  if (cmp_addr(msg->sender.addr, MAC_ADDR) < 3) { return; }

  //debug_print("%02x %02x\n", msg->data.data[POS_ADV_STDMA_OFFSET], msg->data.data[POS_ADV_STDMA_TIMEOUT]);
  // store stdma slot, offset and timeout
  // response data is [header, offset, timeout, data]
  uint8_t temp = stdma_current_slot + msg->data.data[POS_ADV_STDMA_OFFSET];       // next reserved slot
  while (temp > STDMA_SLOTS) { // wrap index on number of slots
    temp = temp - STDMA_SLOTS;
  }

  if (temp != stdma_my_next_slot && msg->data.data[POS_ADV_STDMA_TIMEOUT] > stdma_next_slot_timeout[temp]) {
    stdma_next_slot_timeout[temp]  = msg->data.data[POS_ADV_STDMA_TIMEOUT];
  }

  uint8_t data[128];
  // handle spi
  //data[0] = 0xff - msg->data.len - STDMA_ADV_HEADER_LEN;    // encode msg length in header
  //data[1] = msg->data.data[POS_ADV_TX_STRENGTH];            // tx_strength
  //data[2] = msg->rssi;                                      // rssi

  memcpy(data, msg->data.data + STDMA_ADV_HEADER_LEN, msg->data.len - STDMA_ADV_HEADER_LEN);

  // msg form {header, length, ac_id, msg_id, [msg], crc_a, crc_b}
  // rssi message is [rssi, trans_strength}
  uint8_t rssi_msg[8] = {0x99, 8, data[PPRZ_POS_SENDER_ID], 28, msg->rssi, msg->data.data[POS_ADV_TX_STRENGTH], 0, 0};
  
  uint8_t j = 0, ck_a = 0, ck_b = 0;
  for(j = 1; j < 6; j++){
    ck_a += rssi_msg[j];
    ck_b += ck_a;
  }
  
  rssi_msg[6] = ck_a;
  rssi_msg[7] = ck_b;

  memcpy(data + msg->data.len - STDMA_ADV_HEADER_LEN, rssi_msg, 8);

#ifdef DEBUG
  int i;
  debug_print("recv'd data: ");
  for (i = 0; i < msg->data.len - STDMA_ADV_HEADER_LEN + 8; i++) {
    debug_print("%02x ", data[i]);
  }
  debug_print("\n");
#endif

  server_addr.sin_family = AF_INET;
  ssize_t size = sendto(sock, data, msg->data.len - STDMA_ADV_HEADER_LEN + 8, MSG_DONTWAIT, (struct sockaddr *)&server_addr,
         sizeof(server_addr));
  if(size < 0){
    perror("sending");
  }
}
  
uint8_t skip = 0;
void periodic_fn()
{
  int8_t i = 0;
  int8_t j = 0;
  int8_t k = 0;
// stop broadcasting if I just was
  if (stdma_braodcasting == 1) {
    ble_cmd_gap_set_mode(0, 0); // stop advertisement
    ble_cmd_gap_discover(gap_discover_observation);     // scan for other modules to get rssi values
    stdma_braodcasting = 0;
  }

  if (++stdma_current_slot == STDMA_SLOTS) {      // end of frame
    stdma_current_slot = 0;               // wrap slot counter
    skip = 0;

    // decrement timeout values
    i = 0;
    while (i < STDMA_SLOTS) {
      if (stdma_next_slot_timeout[i] > stdma_slot_timeout[i]) { // copy next statuses to list
        stdma_slot_timeout[i] = stdma_next_slot_timeout[i];
        stdma_slot_status[i] = STDMA_STATE_EXTER_ALLOC;
      }
      stdma_next_slot_timeout[i] = 0;

      if (stdma_slot_timeout[i] > 0) {
        stdma_slot_timeout[i] = stdma_slot_timeout[i] - 1;
      }
      if (stdma_slot_timeout[i] == 0) {
        stdma_slot_status[i] = STDMA_STATE_FREE;  // update slot statuses
      }

      i++;
    }

    // check if my slot is about to expire, if so then broadcast and select new one
    // This logic give the same location double the possible slot options
    if (stdma_slot_timeout[stdma_my_slot] == 0) {
      // find free slots in selection interval
      k = 1;
      stdma_free_slots[0] = stdma_my_slot;
      i = 0 - STDMA_SELECTION_INTERVAL;
      while (i <= STDMA_SELECTION_INTERVAL) {
        j = stdma_my_slot + i;
        // bound in [0,STDMA_SLOTS)
        while (j < 0) {
          j = j + STDMA_SLOTS;
        }
        while (j >= STDMA_SLOTS) {
          j = j - STDMA_SLOTS;
        }

        if (stdma_slot_status[j] == STDMA_STATE_FREE) {
          stdma_free_slots[k] = j;
          k = k + 1;
        }
        i = i + 1;
      }

      // determine next slot using random offset

      stdma_my_next_slot = stdma_free_slots[rand() % k];
      stdma_slot_status[stdma_my_next_slot] = STDMA_STATE_INTER_ALLOC;

      // determine new timeout
      stdma_slot_timeout[stdma_my_next_slot] = rand() % (STDMA_MAX_INTERVAL - STDMA_MIN_INTERVAL) + STDMA_MIN_INTERVAL;
    }
  }

  if (stdma_current_slot == stdma_my_slot && skip == 0) {
    stdma_my_slot = stdma_my_next_slot;
    // set advertisement data
    adv_data[POS_ADV_STDMA_OFFSET] = stdma_my_slot - stdma_current_slot + STDMA_SLOTS;    // offset to next transmission

    while (adv_data[POS_ADV_STDMA_OFFSET] < STDMA_SLOTS + STDMA_MAX_INTERVAL - STDMA_MIN_INTERVAL) {
      adv_data[POS_ADV_STDMA_OFFSET] = adv_data[POS_ADV_STDMA_OFFSET]  + STDMA_SLOTS;
    }

    while (adv_data[POS_ADV_STDMA_OFFSET] > STDMA_SLOTS + STDMA_MAX_INTERVAL - STDMA_MIN_INTERVAL) {
      adv_data[POS_ADV_STDMA_OFFSET] = adv_data[POS_ADV_STDMA_OFFSET] - STDMA_SLOTS;
    }

    if (adv_data[POS_ADV_STDMA_OFFSET] > STDMA_SLOTS) {
      skip = 1;
    }

    adv_data[POS_ADV_STDMA_TIMEOUT] = stdma_slot_timeout[stdma_my_slot];

    adv_data[POS_ADV_LEN] = adv_data_len + STDMA_ADV_DATA_HEADER_LEN;
    ble_cmd_gap_set_adv_data(0, STDMA_ADV_HEADER_LEN + adv_data_len, adv_data);          // Set advertisement data

#ifdef DEBUG
    int g;
    debug_print("advertise data: ");
    for (g = 0; g < STDMA_ADV_HEADER_LEN + adv_data_len; g++) {
      debug_print("%02x ", adv_data[g]);
    }
    debug_print("\n");
#endif

    // broadcast!
    ble_cmd_gap_end_procedure();                        // disable scan

    // enable advertisement
    stdma_braodcasting = 1;
    ble_cmd_gap_set_mode(gap_user_data, gap_scannable_non_connectable);
  }
#ifdef debug
  int s;
  debug_print("timeout: ");
  for (s = 0; s < STDMA_SLOTS; s++) {
    debug_print("%02x ", stdma_slot_timeout[s]);
  }
  debug_print("\n");
#endif
}

void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg) {}
void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg) {}
void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg) {}
void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg) {}
void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg) {}
void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg) {}
void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg) {}

int main(int argc, char *argv[])
{
  char *uart_port = "/dev/ttyACM0";

  if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("Socket");
    exit(1);
  }
  
  int one = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  server_addr.sin_family = PF_INET;
  server_addr.sin_port = htons(5000);
  server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  bzero(&(server_addr.sin_zero), 8);

  client_addr.sin_family = PF_INET;
  client_addr.sin_port = htons(5001);
  client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bzero(&(client_addr.sin_zero), 8);

  if (bind(sock, (struct sockaddr *)&client_addr, sizeof(struct sockaddr)) == -1) {
    perror("Bind failed");
    exit(1);
  }

  bglib_output = output;

  if (uart_open(uart_port)) {
    debug_print("ERROR: Unable to open serial port - %s\n", strerror(errno));
    perror("UART port");
    return 1;
  }

  // Reset dongle to get it into known state
  ble_cmd_system_reset(0);
  uart_close();
  do {
    usleep(500000); // 0.5s
  } while (uart_open(uart_port));

  // initialise STDMA parameters
  memset(stdma_slot_status, STDMA_STATE_FREE, STDMA_SLOTS);
  memset(stdma_slot_timeout, 0, STDMA_SLOTS);
  memset(stdma_next_slot_timeout, 0, STDMA_SLOTS);
  memset(stdma_free_slots, 0, STDMA_SLOTS);

  memset(broadcast_msg, 0, 31);
  broadcast_len = 0;

  stdma_my_slot = 0;
  stdma_my_next_slot = 0;
  stdma_current_slot = 0;
  stdma_braodcasting = 0;

  // Initialize ADV data
  // Flags = LE General Discovery, single mode device (02 01 06) flags for discoverable/connectible
  adv_data[0] = 0x02;               // ad field length = 2 bytes
  adv_data[1] = gap_ad_type_flags;  // ad field type = 0x01 (Flags)
  adv_data[2] = 0x06;               // flags = 0x06, bit 1 General discoverable, bit 2 BR/EDR not supported

  // custom manufacturer
  adv_data[3] = STDMA_ADV_DATA_HEADER_LEN;   // ad field length, minimum 3
  adv_data[4] = 0xff;               // ad field type = 0xFF (Manufacturer Specific Data)
  adv_data[5] = 0xff;               // unknown/prototype Company Identifier Code - octet 2
  adv_data[6] = 0xff;               // unknown/prototype Company Identifier Code - octet 1

  // stdma header
  adv_data[POS_ADV_STDMA_OFFSET] = 0x0;                // stdma offset
  adv_data[POS_ADV_STDMA_TIMEOUT] = 0x0;                // stdma timeout

  // TX power in dBm for BLED USB Dongle
  adv_data[9] = 0x03;
  
  // set fake initial aircraft id
  adv_data[STDMA_ADV_HEADER_LEN + PPRZ_POS_SENDER_ID] = 2;

  // msg data
  adv_data_len = STDMA_ADV_MAX_DATA_LEN;

  ble_cmd_gap_set_adv_data(0, STDMA_ADV_HEADER_LEN + adv_data_len, adv_data);          // Set advertisement data

  // set advertisement interval on all three spi_channels
  // increments of 625us
  // range (0x20 - 0x4000)
  // 0x07: All three channels are used
  // 0x03: Advertisement channels 37 and 38 are used.
  // 0x04: Only advertisement channel 39 is used
  ble_cmd_gap_set_adv_parameters(0x20, 0x28, 0x07);

  // set scan parameters interval/window/use active scanning
  // the scan interval defines the period between restarting a scan, each new scan will switch to a new channel
  // increments of 625us
  // range: 0x4 - 0x4000
  // with active scanning receiver will send a scan response msg
  ble_cmd_gap_set_scan_parameters(0x20, 0x20, 0);     // the values selected should be a multiple of the stdma interval

  /* Intialize random number generator */
  time_t t;
  srand((unsigned) time(&t));

  ble_cmd_gap_discover(gap_discover_observation);   // scan for other modules

  uint8_t counter = 0;
  char buffer[128];
  
  socklen_t len = sizeof(client_addr);
  // Message loop
  while (1) {
    if (read_message(UART_TIMEOUT) > 0) { break; }

    broadcast_len = recvfrom(sock, buffer, 127, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &len);

    // only send message 252, in this config, this msg not sent to ground
    // && spi_data(1+PPRZ_POS_LEN:1) <= STDMA_ADV_MAX_DATA_LEN && spi_data(1+PPRZ_POS_LEN:1) > SPI_DATA_LEN 252
    //if (broadcast_msg[PPRZ_POS_STX] == 0x99 && broadcast_msg[PPRZ_POS_MSG_ID] == 252) {
    if (broadcast_len > 0 && broadcast_len < STDMA_ADV_MAX_DATA_LEN) {
      adv_data_len = broadcast_len;
      memcpy(adv_data + STDMA_ADV_HEADER_LEN, buffer, adv_data_len);
    }
    
    if (!(++counter % 5)) { // 40Hz STDMA
      periodic_fn();
      counter = 0;
    }

    usleep(5000);  // 200 hz loop
  }
  uart_close();

  return 0;
}
