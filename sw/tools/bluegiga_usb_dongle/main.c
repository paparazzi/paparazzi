/*
 * This file is derived from an example Bluegiga ANSI C BGLib demo application: Health Thermometer Collector
 * This is an intermediately program meant to bridge the connection between paparazzi ground station
 * and a paparazzi controlled drone with a bluetooth unit. The bluetooth unit needs to be flashed with
 * the code as found in the "Onboard" folder.
 *
 * Author: Kirk Scheper
 * Email: kirkscheper@gmail.com
 * Last edit: 19/10/15
 */

// Bluegiga ANSI C BGLib demo application: Health Thermometer Collector
//
// Contact: support@bluegiga.com
//
// This is free software distributed under the terms of the MIT license reproduced below.
//
// Copyright (c) 2013 Bluegiga Technologies
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// =============================================================================

/*
BASIC ARCHITECTURAL OVERVIEW:
    The program starts, initializes the dongle to a known state, then starts
    scanning. Each time an advertisement packet is found, a scan response
    event packet is generated. These packets are read by polling the serial
    port to which the BLE(D)11x is attached.

    The basic process is as follows:
      a. Scan for devices
      b. If the desired UUID is found in an ad packet, connect to that device
      c. Search for all "service" descriptors to find the target service handle range
      d. Search through the target service to find the thermometer measurement attribute handle
      e. Enable notifications on the thermometer measurement attribute
      f. Read and display incoming thermometer values until terminated (Ctrl+C)

FUNCTION ANALYSIS:

1. main:
    Parses and validates command-line arguments, then initializes the serial
    port (if directed to) and begins running specified action (serial port
    list, device info, peripheral scan, or direct connection to a known MAC
    address). In the case of a connection it sends commands to cause the
    device to disconnect, stop advertising, and stop scanning (i.e. return to
    a known idle/standby state). Some of these commands will fail since the
    device cannot be doing all of these things at the same time, but this is
    not a problem. This function finishes
    by setting scan parameters and initiating a scan with the "gap_discover"
    command.

2. ble_evt_gap_scan_response:
    Raised during scanning whenever an advertisement packet is detected. The
    data provided includes the MAC address, RSSI, and ad packet data payload.
    This payload includes fields which contain any services being advertised,
    which allows us to scan for a specific service. In this demo, the service
    we are searching for has a standard 16-bit UUID which is contained in the
    "uuid_htm_service" variable. Once a match is found, the script initiates
    a connection request with the "gap_connect_direct" command.

3. ble_evt_connection_status
    Raised when the connection status is updated. This happens when the
    connection is first established, and the "flags" byte will contain 0x05 in
    this instance. However, it will also happen if the connected devices bond
    (i.e. pair), or if encryption is enabled (e.g. with "sm_encrypt_start").
    Once a connection is established, the script begins a service discovery
    with the "attclient_read_by_group_type" command.

4. ble_evt_attclient_group_found
    Raised for each group found during the search started in #3. If the right
    service is found (matched by UUID), then its start/end handle values are
    stored for usage later. We cannot use them immediately because the ongoing
    read-by-group-type procedure must finish first.

5. ble_evt_attclient_find_information_found
    Raised for each attribute found during the search started after the service
    search completes. We look for two specific attributes during this process;
    the first is the unique health thermometer measurement attribute which has
    a standard 16-bit UUID (contained in the "uuid_htm_measurement_characteristic"
    variable), and the second is the corresponding "client characteristic
    configuration" attribute with a UUID of 0x2902. The correct attribute here
    will always be the first 0x2902 attribute after the measurement attribute
    in question. Typically the CCC handle value will be either +1 or +2 from
    the original handle.

6. ble_evt_attclient_procedure_completed
    Raised when an attribute client procedure finishes, which in this script
    means when the "attclient_read_by_group_type" (service search) or the
    "attclient_find_information" (descriptor search) completes. Since both
    processes terminate with this same event, we must keep track of the state
    so we know which one has actually just finished. The completion of the
    service search will (assuming the service is found) trigger the start of
    the descriptor search, and the completion of the descriptor search will
    (assuming the attributes are found) trigger enabling indications on the
    measurement characteristic.

7. ble_evt_attclient_attribute_value
    Raised each time the remote device pushes new data via notifications or
    indications. (Notifications and indications are basically the same, except
    that indications are acknowledged while notifications are not--like TCP vs.
    UDP.) In this script, the remote slave device pushes temperature
    measurements out as indications approximately once per second. These values
    are displayed to the console.

*/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

// command definitions and UART communication implementation
#include "cmd_def.h"
#include "uart.h"

// uncomment the following line to show outgoing/incoming BGAPI packet data
//#define DEBUG

// timeout for serial port read operations
#define UART_TIMEOUT 1000

#include <errno.h>
#include <sys/time.h>
#include <sys/stat.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <time.h>

#include <pthread.h>
#include <signal.h>

#define BUF_SIZE 2048
int sock[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int addr_len, bytes_read, sin_size, bytes_recv;
unsigned char recv_data[BUF_SIZE], data_buf[8][BUF_SIZE], send_buf[8][BUF_SIZE];
struct sockaddr_in send_addr[8], rec_addr[8];
struct hostent *host;
unsigned int send_port , recv_port;

struct timeval tm;
double mytime = 0, old_time = 0;
int count[8] = {0, 0, 0, 0, 0, 0, 0, 0}, send_count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int rssi_count = 0;
int last0 = 0, last1 = 0;

uint8_t connection_interval = 10; // connection interval in ms

int ac_id[8] = { -1, -1, -1, -1, -1, -1, -1, -1};

unsigned int insert_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int extract_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int send_insert_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int send_extract_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0};

#define CLARG_PORT 1
#define CLARG_ACTION 2

#define MAX_DEVICES 8
int connected_devices = 0;
//bd_addr found_devices[MAX_DEVICES];
int connected[] = {0, 0, 0, 0, 0, 0, 0, 0};

int connect_all = 0;
uint8 MAC_ADDR[] = {0x00, 0x00, 0x2d, 0x80, 0x07, 0x00};

// {0x00, 0x00, 0x1e, 0x80, 0x07, 0x00}; // beginning of all dongle adresses

// {0x00,0x00,0x2d,0x80,0x07,0x00};   // beginning of all modules adresses

// list all possible pending actions
enum actions {
  action_none,
  action_scan,
  action_connect,
  action_info,
  action_connect_all,
  action_broadcast
};
enum actions action = action_none;

// list all possible states
typedef enum {
  state_disconnected,             // start/idle state
  state_connecting,               // connection in progress but not established
  state_connected,                // connection established
  state_finding_services,         // listing services (searching for services)
  state_finding_attributes,       // listing attributes (searching for characteristics)
  state_listening_measurements,   // indications enabled, waiting for updates from sensor
  state_finish,                   // application process complete, will exit immediately
  state_last                      // enum tail placeholder
} states;
states state = state_disconnected;

const char *state_names[state_last] = {
  "disconnected",
  "connecting",
  "connected",
  "finding_services",
  "finding_attributes",
  "listening_measurements",
  "finish"
};

#define FIRST_HANDLE 0x0001
#define LAST_HANDLE  0xffff

#define DRONE_SERVICE_UUID      0x77cc
#define DRONE_DATA_UUID         0x9a66
#define DRONE_DATA_CONFIG_UUID  0x2902
#define DRONE_BROADCAST_UUID    0x25ec

uint8 primary_service_uuid[] = {0x00, 0x28};

uint16 drone_handle_start = 0,
       drone_handle_end = 0,
       drone_handle_measurement = 0,
       drone_handle_configuration = 0,
       drone_handle_broadcast = 0;

bd_addr connect_addr;

void usage(char *exe)
{
  fprintf(stderr, "Missing arguments, Possible call structure:\n");
  fprintf(stderr, "[lists available com ports] %s list\n", exe);
  fprintf(stderr, "[Connect direct to one drone]%s <COMx> <mac address> <recieve upd base port> <send udp base port>\n",
          exe);
  fprintf(stderr, "[Scan and connect to all drones] %s <COMx> <scan> <recieve upd base port> <send udp base port>\n",
          exe);
}

/**
 * Change application state
 *
 * @param new_state New state to enter
 */
void change_state(states new_state)
{
#ifdef DEBUG
  fprintf(stderr, "DEBUG: State changed: %s --> %s\n", state_names[state], state_names[new_state]);
#endif
  state = new_state;
}

/**
 * Compare Bluetooth addresses
 *
 * @param first First address
 * @param second Second address
 * @return Zero if addresses are equal, 1 otherwise
 */
int cmp_bdaddr(bd_addr first, bd_addr second)
{
  int i;
  for (i = 0; i < sizeof(bd_addr); i++) {
    if (first.addr[i] != second.addr[i]) { return 1; }
  }
  return 0;
}

/**
 * Compare Bluetooth addresses
 *
 * @param first First address
 * @param second Second address
 * @return returns how many bytes are the same
 */
int cmp_addr(const uint8 first[], const uint8 second[])
{
  int i;
  for (i = 5; i >= 0; i--) {
    if (first[i] != second[i]) { return 5 - i; }
  }
  return 6;
}
/**
 * Display Bluetooth MAC address in hexadecimal notation
 *
 * @param bdaddr Bluetooth MAC address, performs LSB to MSB
 */
void print_bdaddr(bd_addr bdaddr)
{
  fprintf(stderr, "%02x:%02x:%02x:%02x:%02x:%02x",
          bdaddr.addr[5],
          bdaddr.addr[4],
          bdaddr.addr[3],
          bdaddr.addr[2],
          bdaddr.addr[1],
          bdaddr.addr[0]);
}

/**
 * Display raw BGAPI packet in hexadecimal notation
 *
 * @param data1 Fixed-length portion of BGAPI packet (should always be <len1> bytes long)
 * @param hdr BGAPI packet structure pointer
 * @param data Variable-length packet data payload (may be >0 bytes depending on whether uint8array is present in packet)
 */
void print_raw_packet(struct ble_header *hdr, unsigned char *data)
{
  fprintf(stderr, "Incoming packet: ");
  int i;
  for (i = 0; i < sizeof(*hdr); i++) {
    fprintf(stderr, "%02x ", ((unsigned char *)hdr)[i]);
  }
  for (i = 0; i < hdr->lolen; i++) {
    fprintf(stderr, "%02x ", data[i]);
  }
  fprintf(stderr, "\n");
}

/**
 * Send BGAPI packet using UART interface
 *
 * @param len1 Length of fixed portion of packet (always at least 4)
 * @param data1 Fixed-length portion of BGAPI packet (should always be <len1> bytes long)
 * @param len2 Length of variable portion of packet data payload (trailing uint8array or uint16array)
 * @param data2 Variable-length portion of data payload (should always be <len2> bytes long)
 */
void send_api_packet(uint8 len1, uint8 *data1, uint16 len2, uint8 *data2)
{
#ifdef DEBUG
  // display outgoing BGAPI packet
  print_raw_packet((struct ble_header *)data1, data2, 1);
#endif
  if (uart_tx(len1, data1) || uart_tx(len2, data2)) {
    // uart_tx returns non-zero on failure
    fprintf(stderr, "ERROR: Writing to serial port failed\n");
    exit(1);
  }
}

/**
 * Receive BGAPI packet using UART interface
 *
 * @param timeout_ms Milliseconds to wait before timing out on the UART RX operation
 */
int read_api_packet(int timeout_ms)
{
  unsigned char data[256]; // enough for BLE
  struct ble_header hdr;
  int r;

  r = uart_rx(sizeof(hdr), (unsigned char *)&hdr, timeout_ms);
  if (!r) {
    return -1; // timeout
  } else if (r < 0) {
    printf("ERROR: Reading header failed. Error code:%d\n", r);
    return 1;
  }

  if (hdr.lolen) {
    r = uart_rx(hdr.lolen, data, timeout_ms);
    if (r <= 0) {
      printf("ERROR: Reading data failed. Error code:%d\n", r);
      return 1;
    }
  }

  // use BGLib function to create correct ble_msg structure based on the header
  // (header contains command class/ID info, used to identify the right structure to apply)
  const struct ble_msg *msg = ble_get_msg_hdr(hdr);

#ifdef DEBUG
  // display incoming BGAPI packet
  print_raw_packet(&hdr, data, 0);
#endif

  if (!msg) {
    printf("ERROR: Unknown message received\n");
    exit(1);
  }

  // call the appropriate handler function with any payload data
  // (this is what triggers the ble_evt_* and ble_rsp_* functions)
  msg -> handler(data);

  return 0;
}

/**
 * Enables indications on a specified Client Characteristic Configuration attribute
 * Writing the value 0x0002 (little-endian) enables indications.
 * Writing the value 0x0001 (little-endian) enables notifications.
 * @param connection_handle Handle for open connection to use (always "0" in this demo)
 * @param client_configuration_handle 16-bit attribute handle of CCC attribute, used for subscribing to indications
 */
void enable_indications(uint8 connection_handle, uint16 client_configuration_handle)
{
  ble_cmd_sm_encrypt_start(connection_handle, 1);   // encrpyt connection (not strictly required)

  uint8 configuration[] = {0x01, 0x00};       //  {0x01, 0x00} = notify, {0x02, 0x00} indicate
  ble_cmd_attclient_attribute_write(connection_handle, drone_handle_configuration, 2, &configuration);
}

/**
 * "system_get_info" response handler
 * Occurs immediately after requesting system information.
 * (see "system_get_info" command in API reference guide)
 *
 * @param msg Event packet data payload
 */
void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg)
{
  fprintf(stderr, "Build: %u, protocol_version: %u, hardware: ", msg->build, msg->protocol_version);
  switch (msg->hw) {
    case 0x01: printf("BLE112"); break;
    case 0x02: printf("BLE113"); break;
    case 0x03: printf("BLED112"); break;
    default: printf("Unknown");
  }
  fprintf(stderr, "\n");

  if (action == action_info) { change_state(state_finish); }
}

/**
 * "gap_scan_response" event handler
 * Occurs whenever an advertisement packet is detected while scanning
 * (see "gap_discover" command in API reference guide)
 *
 * @param msg Event packet data payload
 */
void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{
  if (action == action_broadcast) {
    fprintf(stderr, "advertisement from: ");
    print_bdaddr(msg->sender);
    fprintf(stderr, " data: ");
    int i;
    for (i = 0; i < msg->data.len; i++) {
      fprintf(stderr, "%02x ", msg->data.data[i]);
    }
    fprintf(stderr, "\n");
    if (sock[0])
      sendto(sock[0], msg->data.data, msg->data.len, MSG_DONTWAIT,
             (struct sockaddr *)&send_addr[0], sizeof(struct sockaddr));
  } else {
    uint8_t i, j;
    char *name = NULL;

    // Check if this device already found, if not add to the list

    if (!connect_all) {
      fprintf(stderr, "New device found: ");

      // Parse data
      for (i = 0; i < msg->data.len;) {
        int8 len = msg->data.data[i++];
        if (!len) { continue; }
        if (i + len > msg->data.len) { break; } // not enough data
        uint8 type = msg->data.data[i++];
        switch (type) {
          case 0x01:
            // flags field
            break;

          case 0x02:
            // partial list of 16-bit UUIDs
          case 0x03:
            // complete list of 16-bit UUIDs
            /*
            for (j = 0; j < len - 1; j += 2)
            {
                // loop through UUIDs 2 bytes at a time
                uint16 test_uuid = msg -> data.data[i + j] + (msg -> data.data[i + j + 1] << 8);
                if (test_uuid == THERMOMETER_SERVICE_UUID)
                {
                    // found the thermometer service UUID in the list of advertised UUIDs!
                    matching_uuid = 1;
                }
            }
            */
            break;

          case 0x04:
            // partial list of 32-bit UUIDs
          case 0x05:
            // complete list of 32-bit UUIDs
            for (j = 0; j < len - 1; j += 4) {
              // loop through UUIDs 4 bytes at a time
              // TODO: test for desired UUID here, if 32-bit UUID
            }
            break;

          case 0x06:
            // partial list of 128-bit UUIDs
          case 0x07:
            // complete list of 128-bit UUIDs
            for (j = 0; j < len - 1; j += 16) {
              // loop through UUIDs 16 bytes at a time
              // TODO: test for desired UUID here, if 128-bit UUID
            }
            break;
          case 0x09:  // device name
            name = malloc(len);
            memcpy(name, msg->data.data + i, len - 1);
            name[len - 1] = '\0';
        }
        i += len - 1;
      }

      print_bdaddr(msg->sender);
      // printf(" RSSI:%d", msg->rssi);

      fprintf(stderr, " Name:");
      if (name) { fprintf(stderr, "%s", name); }
      else { fprintf(stderr, "Unknown"); }
      fprintf(stderr, "\n");

      free(name);
    }

    // automatically connect if responding device has appropriate mac address header
    if (connect_all && cmp_addr(msg->sender.addr, MAC_ADDR) >= 4) {
      fprintf(stderr, "Trying to connect to "); print_bdaddr(msg->sender); fprintf(stderr, "\n");
      //change_state(state_connecting);
      // connection interval unit 1.25ms
      // connection interval must be divisible by number of connection * 2.5ms and larger than minimum (7.5ms)

      // send "gap_connect_direct" command
      // arguments:
      //  - MAC address
      //  - use detected address type (will work with either public or private addressing)
      //  - 6 = 6*1.25ms = 7.5ms minimum connection interval
      //  - 48 = 16*1.25ms = 20ms maximum connection interval
      //  - 100 = 100*10ms = 1000ms supervision timeout
      //  - 9 = 9 connection interval max slave latency
      ble_cmd_gap_connect_direct(&msg->sender.addr, gap_address_type_public, 6, 16, 100, 0);
    }
  }
}

/**
 * "connection_status" event handler
 * Occurs whenever a new connection is established, or an existing one is updated
 *
 * @param msg Event packet data payload
 */
void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg)
{
  // updated connection
  if (msg->flags & connection_parameters_change) {
    fprintf(stderr, "Connection %d parameters updated, interval %fms\n", msg->connection, msg->conn_interval * 1.25);
  }

  // Encrypted previous connection
  else if (msg->flags & connection_encrypted) {
    fprintf(stderr, "Connection with %d is encrypted\n", msg->connection);
  }

  // Connection request completed
  else if (msg->flags & connection_completed) {
    if (msg->connection + 1 > connected_devices) { connected_devices++; }
    //change_state(state_connected);
    connection_interval = msg->conn_interval * 1.25;
    fprintf(stderr, "Connected, nr: %d, connection interval: %d = %fms\n", msg->connection, msg->conn_interval,
            msg->conn_interval * 1.25);
    connected[msg->connection] = 1;

    if (rec_addr[msg->connection].sin_family != AF_INET && send_port && recv_port) {
      if ((sock[msg->connection] = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        exit(1);
      }

      send_addr[msg->connection].sin_family = AF_INET;
      send_addr[msg->connection].sin_port = htons(send_port + msg->connection);
      send_addr[msg->connection].sin_addr = *((struct in_addr *)host->h_addr);
      bzero(&(send_addr[msg->connection].sin_zero), 8);
      sin_size = sizeof(struct sockaddr);

      rec_addr[msg->connection].sin_family = AF_INET;
      rec_addr[msg->connection].sin_port = htons(recv_port + msg->connection);
      rec_addr[msg->connection].sin_addr = *((struct in_addr *)host->h_addr);
      bzero(&(rec_addr[msg->connection].sin_zero), 8);
      sin_size = sizeof(struct sockaddr);

      if (bind(sock[msg->connection], (struct sockaddr *)&rec_addr[msg->connection],
               sizeof(struct sockaddr)) == -1) {
        perror("Bind failed");
        exit(1);
      }
      fprintf(stderr, "Comms port opened on port: %d %d\n", send_port + msg->connection, recv_port + msg->connection);
    }

    // Handle for Drone Data configuration already known
    if (drone_handle_configuration) {
      change_state(state_listening_measurements);
      enable_indications(msg->connection, drone_handle_configuration);
      if (connect_all) {
        ble_cmd_gap_discover(gap_discover_generic);
      }
    }
    // Find primary services
    else {
      change_state(state_finding_services);
      ble_cmd_attclient_read_by_group_type(msg->connection, FIRST_HANDLE, LAST_HANDLE, 2, primary_service_uuid);
    }
  }
}

/**
 * "attclient_group_found" event handler
 * Occurs whenever a GATT group search has returned a new entry (e.g. service)
 * (see "attclient_read_by_group_type" command in API reference guide)
 *
 * @param msg Event packet data payload
 */
void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg)
{
  if (msg->uuid.len == 0) { return; }
  uint16 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];

  if (state == state_finding_services) {
    fprintf(stderr, "length: %d uuid: %x\n", msg->uuid.len, uuid);
  }

  // First data service found
  if (state == state_finding_services && uuid == DRONE_SERVICE_UUID && drone_handle_start == 0) {
    drone_handle_start = msg->start;
    drone_handle_end = msg->end;
  }
}

/**
 * "attclient_procedure_completed" event handler
 * Occurs whenever a service or attribute search has finished, or a few other
 * kinds of GATT client operations. Tracking state is important here since you
 * can end up in this event handler for many reasons.
 *
 * @param msg Event packet data payload
 */
void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg)
{
  if (state == state_finding_services) {
    // Data service not found
    if (drone_handle_start == 0) {
      fprintf(stderr, "No Drone service found\n");
      change_state(state_finish);
    }
    // Find drone service attributes
    else {
      change_state(state_finding_attributes);
      ble_cmd_attclient_find_information(msg->connection, drone_handle_start, drone_handle_end);
    }
  } else if (state == state_finding_attributes) {
    // Client characteristic configuration not found
    if (drone_handle_configuration == 0) {
      fprintf(stderr, "No Client Characteristic Configuration found for Drone Data service\n");
      change_state(state_finish);
    }
    // Enable drone notifications
    else {
      change_state(state_listening_measurements);
      enable_indications(msg->connection, drone_handle_configuration);
      if (connect_all) {
        ble_cmd_gap_discover(gap_discover_generic);
      }
    }
  }

  // preivous message parsed on device, device now ready for next message
  //else if (state == state_listening_measurements) {
  //  extract_idx[msg->connection] = (extract_idx[msg->connection] + bt_msg_len[msg->connection]) % BUF_SIZE;
  //  send_msg(msg->connection, 1);
  //}
}

/**
 * "attclient_find_information_found" event handler
 * Occurs whenever an information search has returned a new entry (e.g. attribute)
 * (see "attclient_find_information" in API reference guide)
 *
 * @param msg Event packet data payload
 */
void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg)
{
//TODO: add different rule for 16
  if (msg->uuid.len == 2 || msg->uuid.len == 16) {
    uint32 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];
    if (uuid == DRONE_DATA_CONFIG_UUID) {
      drone_handle_configuration = msg->chrhandle;
    } else if (uuid == DRONE_DATA_UUID) {
      drone_handle_measurement = msg->chrhandle;
    } else if (uuid == DRONE_BROADCAST_UUID) {
      drone_handle_broadcast = msg->chrhandle;
    }
  }
}

/**
 * "attclient_attribute_value" event handler
 * Occurs whenever the remote GATT server has pushed a new value to us via notifications or indications
 *
 * @param msg Event packet data payload
 */
void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg)
{
  /*
    gettimeofday(&tm, NULL); //Time zone struct is obsolete, hence NULL
    mytime = (double)tm.tv_sec + (double)tm.tv_usec / 1000000.0;

    count[msg->connection]++;
    if(mytime - old_time > 1){
      int i = 0;
      while(ac_id[i]!=-1 && i < 8){
        fprintf(stderr,"Connection %d\nspeed: %dbps message frequency = %dHz\n",i,count[i]*20*8,count[i]);
        fprintf(stderr,"send speed: %dbps buffer size: %d\n",send_count[i],(insert_idx[i] - extract_idx[i])%BUF_SIZE);
        fprintf(stderr,"extract idx: %d, insert idx: %d\n",extract_idx[i], insert_idx[i]);

        send_count[i] = 0;
        count[i] = 0;
        i++;
      }
      old_time = mytime;
    }*/

  if (ac_id[msg->connection] == -1 && msg->value.data[0] == 0x99) {
    ac_id[msg->connection] = msg->value.data[2];
    fprintf(stderr, "\nAC_ID = %d\n", ac_id[msg->connection]);
  }

  //memcpy(&send_buf[msg->connection][send_insert_idx[msg->connection]], msg->value.data, msg->value.len);
  //send_insert_idx[msg->connection] = (send_insert_idx[msg->connection] + msg->value.len)%BUF_SIZE;
  if (sock[msg->connection])
    sendto(sock[msg->connection], msg->value.data, msg->value.len, MSG_DONTWAIT,
           (struct sockaddr *)&send_addr[msg->connection], sizeof(struct sockaddr));

}

/**
 * "connection_disconnected" event handler
 * Occurs whenever the BLE connection to the peripheral device has been terminated
 *
 * @param msg Event packet data payload
 */
void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg)
{
  // reset connection parameters
  connected[msg->connection] = 0;

  extract_idx[msg->connection] = 0;
  insert_idx[msg->connection] = 0;

  // remove found device from list
  //change_state(state_disconnected);
  fprintf(stderr, "Connection %d terminated\n" , msg->connection);
  if (!connect_all) {
    ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 6, 16, 100, 0);
    fprintf(stderr, "Trying to reconnection to ");
    print_bdaddr(connect_addr);
  }
  //change_state(state_connecting);

  //change_state(state_finish);
}

void ble_rsp_connection_update(const struct ble_msg_connection_update_rsp_t *msg)
{
  fprintf(stderr, "Connection update result from %d: %x\n", msg->connection, msg->result);
}

void ble_rsp_system_address_get(const struct ble_msg_system_address_get_rsp_t *msg)
{
  fprintf(stderr, "My address: ");
  print_bdaddr(msg->address);
  fprintf(stderr, "\n");
}

void *send_msg()
{
  fprintf(stderr, "Bluegiga comms thread started\n");
  uint8_t device = 0, bt_msg_len = 0;
  uint16_t diff = 0;
  while (state != state_finish) {
    if (action == action_broadcast) {
      diff = (insert_idx[0] - extract_idx[0] + BUF_SIZE) % BUF_SIZE;
      if (diff) {
        ble_cmd_gap_end_procedure();
        bt_msg_len = diff < 31 ? diff : 31;
        //ble_cmd_attclient_attribute_write(device, drone_handle_measurement, bt_msg_len[device], &data_buf[device][extract_idx[device]]);

        ble_cmd_gap_set_adv_data(0, bt_msg_len, &data_buf[0][extract_idx[0]]);
        ble_cmd_gap_set_mode(gap_user_data, gap_non_connectable);    //gap_set_mode($84, gap_scannable_non_connectable)
        extract_idx[device] = (extract_idx[device] + bt_msg_len) % BUF_SIZE;

        usleep(10000); // advertisement interval set at 320ms so pause for shorter before turning off
        ble_cmd_gap_set_mode(0, 0);   // stop advertising

        ble_cmd_gap_discover(gap_discover_observation);   // return to listening
        usleep(5000);
      }
    } else {
      device = 0;
      while (ac_id[device] != -1 && device < 8) {
        diff = (insert_idx[device] - extract_idx[device] + BUF_SIZE) % BUF_SIZE;
        if (diff) {
          bt_msg_len = diff < 19 ? diff : 19; // msg length in max 20 but one header byte added on bluegiga to lisa
          //if (bt_msg_len > 18)
          //  fprintf(stderr,"Long msg: %d, buff size: %d\n", bt_msg_len, diff);
          //ble_cmd_attclient_attribute_write(device, drone_handle_measurement, bt_msg_len[device], &data_buf[device][extract_idx[device]]);

          ble_cmd_attclient_write_command(device, drone_handle_measurement, bt_msg_len, &data_buf[device][extract_idx[device]]);
          extract_idx[device] = (extract_idx[device] + bt_msg_len) % BUF_SIZE;
        }
        device++;
      } // next device
      usleep(connection_interval * 1000); // send messages at max intervals of the connection interval
    } // repeat
  }
  pthread_exit(NULL);
}

void *recv_paparazzi_comms()
{
  fprintf(stderr, "Paparazzi comms thread started\n");
  unsigned char device = 0;
  while (state != state_finish) {
    if (state == state_listening_measurements) {
      if (action == action_broadcast) {
        if (sock[0]) {
          bytes_recv = recvfrom(sock[0], recv_data, BUF_SIZE, MSG_DONTWAIT, (struct sockaddr *)&rec_addr[0],
                                (socklen_t *)&sin_size);
          if (bytes_recv > 0) {
            send_count[0] += bytes_recv;
            memcpy(&data_buf[0][insert_idx[0]], recv_data, bytes_recv);

            insert_idx[0] = (insert_idx[0] + bytes_recv) % BUF_SIZE;
          }
        }
      } else {
        device = 0;
        while (ac_id[device] != -1 && device < 8) {
          if (connected[device] && sock[device]) {
            bytes_recv = recvfrom(sock[device], recv_data, BUF_SIZE, MSG_DONTWAIT, (struct sockaddr *)&rec_addr[device],
                                  (socklen_t *)&sin_size);
            if (bytes_recv > 0) { // TODO: can overtake extract!
              send_count[device] += bytes_recv;
              memcpy(&data_buf[device][insert_idx[device]], recv_data, bytes_recv);

              insert_idx[device] = (insert_idx[device] + bytes_recv) % BUF_SIZE;
            }
          }
          device++;
        }
      }
    }
    usleep(20000);  // assuming connection interval 10ms, give a bit of overhead
  }
  pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
  pthread_t threads[2];

  send_port = recv_port = 0;

  char *uart_port = "";
  gettimeofday(&tm, NULL);

  old_time = (double)tm.tv_sec + (double)tm.tv_usec / 1000000.0;

  //ble_cmd_sm_set_bondable_mode(1);

  host = (struct hostent *) gethostbyname((char *)"127.0.0.1");

  // Not enough command-line arguments
  if (argc <= CLARG_PORT) {
    usage(argv[0]);
    return 1;
  }

  // COM port argument
  if (argc > CLARG_PORT) {
    if (strcmp(argv[CLARG_PORT], "list") == 0) {
      uart_list_devices();
      return 1;
    } else if (strcmp(argv[CLARG_PORT], "-h") == 0 || strcmp(argv[CLARG_PORT], "--help") == 0) {
      usage(argv[0]);
    } else {
      uart_port = argv[CLARG_PORT];
    }
  }

  // Action argument
  if (argc > CLARG_ACTION) {
    int i;
    for (i = 0; i < strlen(argv[CLARG_ACTION]); i++) {
      argv[CLARG_ACTION][i] = tolower(argv[CLARG_ACTION][i]);
    }

    if (strcmp(argv[CLARG_ACTION], "scan") == 0) {
      action = action_scan;
    } else if (strcmp(argv[CLARG_ACTION], "info") == 0) {
      action = action_info;
    } else if (strcmp(argv[CLARG_ACTION], "broadcast") == 0) {
      action = action_broadcast;
      if (argc > CLARG_ACTION + 2) {
        send_port = atoi(argv[CLARG_ACTION + 1]);
        recv_port = atoi(argv[CLARG_ACTION + 2]);
      } else {
        usage(argv[0]);
        return 1;
      }
    } else if (strcmp(argv[CLARG_ACTION], "all") == 0) {
      connect_all = 1;
      action = action_scan;
      if (argc > CLARG_ACTION + 2) {
        send_port = atoi(argv[CLARG_ACTION + 1]);
        recv_port = atoi(argv[CLARG_ACTION + 2]);
      } else {
        usage(argv[0]);
        return 1;
      }
    } else {
      int i;
      short unsigned int addr[6];
      if (sscanf(argv[CLARG_ACTION],
                 "%02hx:%02hx:%02hx:%02hx:%02hx:%02hx",
                 &addr[5],
                 &addr[4],
                 &addr[3],
                 &addr[2],
                 &addr[1],
                 &addr[0]) == 6) {

        for (i = 0; i < 6; i++) {
          connect_addr.addr[i] = addr[i];
        }
        action = action_connect;
        if (argc > CLARG_ACTION + 2) {
          send_port = atoi(argv[CLARG_ACTION + 1]);
          recv_port = atoi(argv[CLARG_ACTION + 2]);
        } else {
          usage(argv[0]);
          return 1;
        }
      }
    }
  }
  if (action == action_none) {
    usage(argv[0]);
    return 1;
  }

  // set BGLib output function pointer to "send_api_packet" function
  bglib_output = send_api_packet;

  if (uart_open(uart_port)) {
    fprintf(stderr, "ERROR: Unable to open serial port - %s\n", strerror(errno));
    return 1;
  }

  // Reset dongle to get it into known state
  ble_cmd_system_reset(0);

#if 0 // very soft "reset"
  // close current connection, stop scanning, stop advertising
  // (returns BLE device to a known state without a hard reset)
  ble_cmd_connection_disconnect(0);
  ble_cmd_gap_set_mode(0, 0);
  ble_cmd_gap_end_procedure();

#else // full reset
  // reset BLE device to get it into known state
  ble_cmd_system_reset(0);

  // close the serial port, since reset causes USB to re-enumerate
  uart_close();

  // wait until USB re-enumerates and we can re-open the port
  // (not necessary if using a direct UART connection)
  do {
    usleep(500000); // 0.5s
  } while (uart_open(uart_port));
#endif

  // get the mac address of the dongle
  ble_cmd_system_address_get();

  // Execute action
  if (action == action_scan) {
    ble_cmd_gap_discover(gap_discover_generic);
  } else if (action == action_info) {
    ble_cmd_system_get_info();
  } else if (action == action_connect) {
    fprintf(stderr, "Trying to connect\n");
    change_state(state_connecting);
    ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 6, 16, 100, 0);
  } else if (action == action_broadcast) {
    ble_cmd_gap_set_adv_parameters(0x200, 0x200,
                                   0x07);    // advertise interval scales 625us, min, max, channels (0x07 = 3, 0x03 = 2, 0x04 = 1)
  }

  pthread_create(&threads[0], NULL, send_msg, NULL);
  pthread_create(&threads[1], NULL, recv_paparazzi_comms, NULL);

  // Message loop
  while (state != state_finish) {
    if (read_api_packet(UART_TIMEOUT) > 0) { break; }
  }

  change_state(state_finish);

  ble_cmd_gap_end_procedure();

  uart_close();

  pthread_exit(NULL);
}
