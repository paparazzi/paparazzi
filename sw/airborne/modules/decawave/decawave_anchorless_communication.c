/*
 * Serial_Communication.c
 *
 *  Created on: Jul 25, 2017
 *      Author: Steven van der Helm
 */

/*
 * Copyright (C) C. DW
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/decawave_anchorless_communication.h"
 * @author S. vd H, C. DW
 *
  Decawave DWM1000 module serial communication for use in anchorless network where the UWB modules are attached to MAVs and need to communicate
  on-board values with each-other (for purposes such as relative localization, co-ordination, or collision avoidance).
  This module must be used together with the Decawave DWM1000 running the appropriate Serial Communication code, which can be flashed on the Arduino board.
  The arduino library can be found at:
    https://github.com/StevenH2812/arduino-dw1000/tree/UWB_onboard
  The example file to flash the Arduino Micro can be found in
    examples/UWB_localization_v1_0/UWB_localization_v1_0.ino
 */

#include "decawave_anchorless_communication.h"
#include "modules/datalink/telemetry.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include <stdio.h>

#define UWB_SERIAL_PORT (&((SERIAL_UART).device))
struct link_device *external_device = UWB_SERIAL_PORT;

// Some meta data for serial communication
#define UWB_SERIAL_COMM_MAX_MESSAGE 20
#define UWB_SERIAL_COMM_END_MARKER 255
#define UWB_SERIAL_COMM_SPECIAL_BYTE 253
#define UWB_SERIAL_COMM_START_MARKER 254
#define UWB_SERIAL_COMM_NODE_STATE_SIZE 7

#define UWB_SERIAL_COMM_NUM_NODES 3 // How many nodes actually are in the network
#define UWB_SERIAL_COMM_DIST_NUM_NODES UWB_SERIAL_COMM_NUM_NODES-1  // How many distant nodes are in the network (one less than the toal number of nodes)

// Serial message

#define UWB_SERIAL_COMM_RANGE 0
#define UWB_SERIAL_COMM_VX 1
#define UWB_SERIAL_COMM_VY 2
#define UWB_SERIAL_COMM_Z 3
#define UWB_SERIAL_COMM_AX 4
#define UWB_SERIAL_COMM_AY 5
#define UWB_SERIAL_COMM_YAWR 6

struct nodeState {
  uint8_t nodeAddress;
  float r;
  float vx;
  float vy;
  float z;
  float ax;
  float ay;
  float yawr;
  bool state_updated[UWB_SERIAL_COMM_NODE_STATE_SIZE];
};

static struct nodeState states[UWB_SERIAL_COMM_DIST_NUM_NODES];

/**
 * Function that is called when over the serial a new state value from a remote node is received
 */
static void handleNewStateValue(uint8_t nodeIndex, uint8_t msg_type, float value)
{
  struct nodeState *node = &states[nodeIndex];
  switch (msg_type) {
    case UWB_SERIAL_COMM_RANGE :
      node->r = value;
      node->state_updated[UWB_SERIAL_COMM_RANGE] = true;
      break;
    case UWB_SERIAL_COMM_VX :
      node->vx = value;
      node->state_updated[UWB_SERIAL_COMM_VX] = true;
      break;
    case UWB_SERIAL_COMM_VY :
      node->vy = value;
      node->state_updated[UWB_SERIAL_COMM_VY] = true;
      break;
    case UWB_SERIAL_COMM_Z :
      node->z = value;
      node->state_updated[UWB_SERIAL_COMM_Z] = true;
      break;
    case UWB_SERIAL_COMM_AX :
      node->ax = value;
      node->state_updated[UWB_SERIAL_COMM_AX] = true;
      break;
    case UWB_SERIAL_COMM_AY :
      node->ay = value;
      node->state_updated[UWB_SERIAL_COMM_AY] = true;
      break;
    case UWB_SERIAL_COMM_YAWR :
      node->yawr = value;
      node->state_updated[UWB_SERIAL_COMM_YAWR] = true;
      break;
  }
}

/**
 * Function for decoding the high bytes of received serial data and saving the message.
 * Since the start and end marker could also be regular payload bytes (since they are simply the values
 * 254 and 255, which could also be payload data) the payload values 254 and 255 have been encoded
 * as byte pairs 253 1 and 253 2 respectively. Value 253 itself is encoded as 253 0.
 *  This function will decode these back into values the original payload values.
 */
static void decodeHighBytes(uint8_t bytes_received, uint8_t *received_message)
{
  static uint8_t receive_buffer[4];
  float tempfloat;
  uint8_t data_received_count = 0;
  uint8_t this_address = received_message[1];
  uint8_t msg_from = received_message[2];
  uint8_t msg_type = received_message[3];
  uint8_t nodeIndex = msg_from - 1 - (uint8_t)(this_address < msg_from);
  for (uint8_t i = 4; i < bytes_received - 1; i++) {
    // Skip the begin marker (0), this address (1), remote address (2), message type (3), and end marker (bytes_received-1)
    uint8_t var_byte = received_message[i];
    if (var_byte == UWB_SERIAL_COMM_SPECIAL_BYTE) {
      i++;
      var_byte = var_byte + received_message[i];
    }
    if (data_received_count <= 4) {
      receive_buffer[data_received_count] = var_byte;
    }
    data_received_count++;
  }
  if (data_received_count == 4) {
    // Move memory from integer buffer to float variable
    memcpy(&tempfloat, &receive_buffer, 4);
    // Set the variable to the appropriate type and store it in state
    handleNewStateValue(nodeIndex, msg_type, tempfloat);
  }
}

/**
 * Function that encodes the high bytes of the serial data to be sent.
 * Start and end markers are reserved values 254 and 255. In order to be able to send these values,
 * the payload values 253, 254, and 255 are encoded as 2 bytes, respectively 253 0, 253 1, and 253 2.
 */
static void encodeHighBytes(uint8_t *send_data, uint8_t msg_size, uint8_t *data_send_buffer, uint8_t *data_total_send)
{
  uint8_t data_send_count = msg_size;
  *data_total_send = 0;
  for (uint8_t i = 0; i < data_send_count; i++) {
    if (send_data[i] >= UWB_SERIAL_COMM_SPECIAL_BYTE) {
      data_send_buffer[*data_total_send] = UWB_SERIAL_COMM_SPECIAL_BYTE;
      (*data_total_send)++;
      data_send_buffer[*data_total_send] = send_data[i] - UWB_SERIAL_COMM_SPECIAL_BYTE;
    } else {
      data_send_buffer[*data_total_send] = send_data[i];
    }
    (*data_total_send)++;
  }
}

/**
 * Function that will send a float over serial. The actual message that will be sent will have
 * a start marker, the message type, 4 bytes for the float, and the end marker.
 */
static void sendFloat(uint8_t msg_type, float data)
{
  static uint8_t data_send_buffer[UWB_SERIAL_COMM_MAX_MESSAGE];
  static uint8_t data_total_send = 0;

  // Make bytes of the float
  uint8_t floatbyte[4];
  memcpy(floatbyte, &data, 4);
  encodeHighBytes(floatbyte, 4, data_send_buffer, &data_total_send);

  UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, UWB_SERIAL_COMM_START_MARKER);
  UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, msg_type);

  for (uint8_t i = 0; i < data_total_send; i++) {
    UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, data_send_buffer[i]);
  }

  UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, UWB_SERIAL_COMM_END_MARKER);
}

/**
 * Helper function that sets the boolean that tells whether a remote drone has a new state update to false.
 */
static void setNodeStatesFalse(uint8_t index)
{
  for (uint8_t j = 0; j < UWB_SERIAL_COMM_NODE_STATE_SIZE; j++) {
    states[index].state_updated[j] = false;
  }
}

/**
 * This function checks if all the states of all the distant nodes have at least once been updated.
 * If all the states are updated, then do something with it! AKA CALLBACK TO MARIO
 */
static void checkStatesUpdated(void)
{
  bool checkbool;
  for (uint8_t i = 0; i < UWB_SERIAL_COMM_DIST_NUM_NODES; i++) {
    checkbool = true;
    for (uint8_t j = 0; j < UWB_SERIAL_COMM_NODE_STATE_SIZE; j++) {
      checkbool = checkbool && states[i].state_updated[j];
    }
    if (checkbool) {
      AbiSendMsgUWB_COMMUNICATION(UWB_COMM_ID, i, states[i].r, states[i].vx, states[i].vy, states[i].z, states[i].ax, states[i].ay, states[i].yawr);
      setNodeStatesFalse(i);
    }
  }
}

/**
 * Function for receiving serial data.
 * Only receives serial data that is between the start and end markers. Discards all other data.
 * Stores the received data in received_message, and after decodes the high bytes and copies the final
 * message to the corresponding message in _messages.
 */
static void getSerialData(uint8_t *bytes_received)
{
  static bool in_progress = false;
  static uint8_t var_byte;
  static uint8_t received_message[UWB_SERIAL_COMM_MAX_MESSAGE];

  while (external_device->char_available(external_device->periph)) {
    var_byte = UWB_SERIAL_PORT->get_byte(UWB_SERIAL_PORT->periph);

    if (var_byte == UWB_SERIAL_COMM_START_MARKER) {
      (*bytes_received) = 0;
      in_progress = true;
    }

    if (in_progress) {
      if ((*bytes_received) < UWB_SERIAL_COMM_MAX_MESSAGE - 1) {
        received_message[*bytes_received] = var_byte;
        (*bytes_received)++;
      } else {
        in_progress = false;
      }
    }

    if (var_byte == UWB_SERIAL_COMM_END_MARKER) {
      in_progress = false;
      decodeHighBytes(*bytes_received, received_message);
    }
  }
}

/**
 * Initialization functio
 */
void decawave_anchorless_communication_init(void)
{
  // Set all nodes to false
  for (uint8_t i = 0; i < UWB_SERIAL_COMM_DIST_NUM_NODES; i++) {
    setNodeStatesFalse(i);
  }
}

/**
 * This function periodically sends state data over the serial (which is received by the arduino)
 */
void decawave_anchorless_communication_periodic(void)
{
  // TODO: Right now floats are sent individually, but it would be nice to send all at once (requires integrating with UWB/Arduino side as well).
  sendFloat(UWB_SERIAL_COMM_VX, stateGetSpeedEnu_f()->y);
  sendFloat(UWB_SERIAL_COMM_VY, stateGetSpeedEnu_f()->x);
  sendFloat(UWB_SERIAL_COMM_Z, stateGetPositionEnu_f()->z);
  sendFloat(UWB_SERIAL_COMM_AX, stateGetAccelNed_f()->x);
  sendFloat(UWB_SERIAL_COMM_AY, stateGetAccelNed_f()->y);
  sendFloat(UWB_SERIAL_COMM_YAWR, stateGetBodyRates_f()->r);
}

/**
 * Event function currently checks for serial data and whether an update of states is available for a distant drone.
 * If these cases are true, then actions are taken.
 */
void decawave_anchorless_communication_event(void)
{
  static uint8_t bytes_received;
  getSerialData(&bytes_received);
  checkStatesUpdated();
}
