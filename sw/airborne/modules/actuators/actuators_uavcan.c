/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/actuators/actuators_uavcan.c
 * UAVCan actuators using RAWCOMMAND message and ESC_STATUS telemetry
 *
 */

#include "actuators_uavcan.h"
#include "modules/energy/electrical.h"
#include "math/pprz_random.h"
#include "modules/core/abi.h"
#include "modules/actuators/actuators.h"


#include "uavcan.equipment.esc.Status.h"
#include "uavcan.equipment.esc.RawCommand.h"
#include "uavcan.equipment.actuator.Status.h"
#include "uavcan.equipment.actuator.ArrayCommand.h"
#include "uavcan.equipment.device.Temperature.h"

/* By default enable the usage of the current sensing in the ESC telemetry */
#ifndef UAVCAN_ACTUATORS_USE_CURRENT
#define UAVCAN_ACTUATORS_USE_CURRENT TRUE
#endif

/* uavcan ESC status telemetry structure */
struct actuators_uavcan_telem_t {
  bool set;
  uint8_t node_id;
  float timestamp;
  float voltage;
  float current;
  float temperature;
  float temperature_dev;
  int32_t rpm;
  uint32_t energy;
  float position;
};

/* The transmitted actuator values */
#ifdef SERVOS_UAVCAN1_NB
int16_t actuators_uavcan1_values[SERVOS_UAVCAN1_NB];
#endif
#ifdef SERVOS_UAVCAN2_NB
int16_t actuators_uavcan2_values[SERVOS_UAVCAN2_NB];
#endif
#ifdef SERVOS_UAVCAN1CMD_NB
int16_t actuators_uavcan1cmd_values[SERVOS_UAVCAN1CMD_NB];
#endif
#ifdef SERVOS_UAVCAN2CMD_NB
int16_t actuators_uavcan2cmd_values[SERVOS_UAVCAN2CMD_NB];
#endif

/* Set the actual telemetry length (ID's from actuators can't collide with the command version) */
#if SERVOS_UAVCAN1CMD_NB > SERVOS_UAVCAN1_NB
#define UAVCAN1_TELEM_NB SERVOS_UAVCAN1CMD_NB
static struct actuators_uavcan_telem_t uavcan1_telem[SERVOS_UAVCAN1CMD_NB] = {0};
#elif defined(SERVOS_UAVCAN1_NB)
#define UAVCAN1_TELEM_NB SERVOS_UAVCAN1_NB
static struct actuators_uavcan_telem_t uavcan1_telem[SERVOS_UAVCAN1_NB] = {0};
#endif

#if SERVOS_UAVCAN2CMD_NB > SERVOS_UAVCAN2_NB
#define UAVCAN2_TELEM_NB SERVOS_UAVCAN2CMD_NB
static struct actuators_uavcan_telem_t uavcan2_telem[SERVOS_UAVCAN2CMD_NB] = {0};
#elif defined(SERVOS_UAVCAN2_NB)
#define UAVCAN2_TELEM_NB SERVOS_UAVCAN2_NB
static struct actuators_uavcan_telem_t uavcan2_telem[SERVOS_UAVCAN2_NB] = {0};
#endif

/* UNUSED value for CMD */
#define UAVCAN_CMD_UNUSED (MIN_PPRZ-1)

/* private variables */
static bool actuators_uavcan_initialized = false;
static uavcan_event esc_status_ev;
static uavcan_event actuator_status_ev;
static uavcan_event device_temperature_ev;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static uint8_t old_idx = 0;
static uint8_t esc_idx = 0;
static struct actuators_uavcan_telem_t *actuators_uavcan_next_telem(void) {
  // Randomness added for multiple  transport devices
  uint8_t add_idx = 0;
  if (rand_uniform() > 0.02) {
    add_idx = 1;
  }

  // Find the next set telemetry
  uint8_t offset = 0;
#ifdef UAVCAN1_TELEM_NB
  for(uint8_t i = esc_idx - offset; i < UAVCAN1_TELEM_NB; i++) {
    if(uavcan1_telem[i].set) {
      old_idx = i + offset;
      esc_idx = i + offset + add_idx;
      return &uavcan1_telem[i];
    } else {
      esc_idx = i + offset + 1;
    }
  }
  offset += UAVCAN1_TELEM_NB;
#endif
#ifdef UAVCAN2_TELEM_NB
  for(uint8_t i = esc_idx - offset; i < UAVCAN2_TELEM_NB; i++) {
    if(uavcan2_telem[i].set) {
      old_idx = i + offset;
      esc_idx = i + offset + add_idx;
      return &uavcan2_telem[i];
    } else {
      esc_idx = i + offset + 1;
    }
  }
  offset += UAVCAN2_TELEM_NB;
#endif

  // Going round or no telemetry found
  esc_idx = 0;
  return NULL;
}

static void actuators_uavcan_send_esc(struct transport_tx *trans, struct link_device *dev)
{
  // Find the correct telemetry (Try twice if going round)
  struct actuators_uavcan_telem_t *telem = actuators_uavcan_next_telem();
  if(telem == NULL) {
    telem = actuators_uavcan_next_telem();
  }

  // Safety check (no telemetry received 2 times)
  if (telem == NULL) {
    return;
  }

  float power = telem->current * telem->voltage;
  float rpm = telem->rpm;
  float energy = telem->energy;
  pprz_msg_send_ESC(trans, dev, AC_ID, &telem->current, &electrical.vsupply, &power,
                    &rpm, &telem->voltage, &energy, &telem->temperature, &telem->temperature_dev, &telem->node_id, &old_idx);
}
#endif

static struct actuators_uavcan_telem_t *get_actuator_telem(struct uavcan_iface_t *iface, uint8_t idx) {
  struct actuators_uavcan_telem_t *telem = NULL;

#ifdef UAVCAN1_TELEM_NB
  if (iface == &uavcan1 && idx < UAVCAN1_TELEM_NB) {
    telem = &uavcan1_telem[idx];
  }
#endif
#ifdef UAVCAN2_TELEM_NB
  if (iface == &uavcan2 && idx < UAVCAN2_TELEM_NB) {
    telem = &uavcan2_telem[idx];
  }
#endif

  return telem;
}

static uint8_t get_actuator_idx(struct uavcan_iface_t *iface, uint8_t idx) {
  uint8_t actuator_idx = 255;

#ifdef UAVCAN1_TELEM_NB
  if(iface == &uavcan1) {
#ifdef SERVOS_UAVCAN1_NB
    // First try as RAW COMMAND
    actuator_idx = get_servo_idx_UAVCAN1(idx);
#endif
#ifdef SERVOS_UAVCAN1CMD_NB
    // Then try as ACTUATOR COMMAND
    if(idx < SERVOS_UAVCAN1CMD_NB && actuators_uavcan1cmd_values[idx] != UAVCAN_CMD_UNUSED) {
      actuator_idx = get_servo_idx_UAVCAN1CMD(idx);
    }
#endif
  }
#endif
#ifdef UAVCAN2_TELEM_NB
  if(iface == &uavcan2) {
#ifdef SERVOS_UAVCAN2_NB
    // First try as RAW COMMAND
    actuator_idx = get_servo_idx_UAVCAN2(idx);
#endif
#ifdef SERVOS_UAVCAN2CMD_NB
    // Then try as ACTUATOR COMMAND
    if(idx < SERVOS_UAVCAN2CMD_NB && actuators_uavcan2cmd_values[idx] != UAVCAN_CMD_UNUSED) {
      actuator_idx = get_servo_idx_UAVCAN2CMD(idx);
    }
#endif
  }
#endif

  return actuator_idx;
}

/**
 * Whevener an ESC_STATUS message from the EQUIPMENT group is received
 */
static void actuators_uavcan_esc_status_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer)
{
  // Decode the message
  struct uavcan_equipment_esc_Status msg;
  if(uavcan_equipment_esc_Status_decode(transfer, &msg)) {
    return; // Decoding error
  }

  // Get the correct telemetry
  struct actuators_uavcan_telem_t *telem = get_actuator_telem(iface, msg.esc_index);
  if(telem == NULL) {
    return;
  }

  // Set the telemetry (FIXME: copy over fully)
  telem->set = true;
  telem->node_id = transfer->source_node_id;
  telem->timestamp = get_sys_time_float();
  telem->energy = msg.error_count;
  telem->voltage = msg.voltage;
  telem->current = msg.current;
  telem->temperature = msg.temperature;
  telem->rpm = msg.rpm;

  /* Specification says Kelvin, but some are transmitting in Celsius */
  if (telem->temperature > 230.f) {
    telem->temperature -= 273.15;
  }

  // Feedback ABI RPM messages
  struct act_feedback_t feedback = {0};
  feedback.idx = get_actuator_idx(iface, msg.esc_index);
  feedback.rpm = telem->rpm;
  feedback.set.rpm = true;

  // Send ABI message
  AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);

#if UAVCAN_ACTUATORS_USE_CURRENT
  // Update total current based on ESC telemetry
  electrical.current = 0;
#ifdef UAVCAN1_TELEM_NB
  for (uint8_t i = 0; i < UAVCAN1_TELEM_NB; ++i) {
    electrical.current += uavcan1_telem[i].current;
  }
#endif
#ifdef UAVCAN2_TELEM_NB
  for (uint8_t i = 0; i < UAVCAN2_TELEM_NB; ++i) {
    electrical.current += uavcan2_telem[i].current;
  }
#endif
#endif
}

/**
 * Whevener an ACTUATOR_STATUS message from the EQUIPMENT group is received
 */
static void actuators_uavcan_actuator_status_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer)
{
  // Decode the message
  struct uavcan_equipment_actuator_Status msg;
  if(uavcan_equipment_actuator_Status_decode(transfer, &msg)) {
    return; // Decoding error
  }

  // Get the correct telemetry
  struct actuators_uavcan_telem_t *telem = get_actuator_telem(iface, msg.actuator_id);
  if(telem == NULL) {
    return;
  }

  //telem[msg.actuator_id].set = true;
  telem->position = msg.position;

  // Feedback ABI position messages
  struct act_feedback_t feedback = {0};
  feedback.idx = get_actuator_idx(iface, msg.actuator_id);
  feedback.position = telem->position;
  feedback.set.position = true;

  // Send ABI message
  AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
}

/**
 * Whevener an DEVICE_TEMPERATURE message from the EQUIPMENT group is received
 */
static void actuators_uavcan_device_temperature_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer)
{
  // Decode the message
  struct uavcan_equipment_device_Temperature msg;
  if(uavcan_equipment_device_Temperature_decode(transfer, &msg)) {
    return; // Decoding error
  }

  // Get the correct telemetry
  struct actuators_uavcan_telem_t *telem = get_actuator_telem(iface, msg.device_id);
  if(telem == NULL) {
    return;
  }

  telem->set = true;
  telem->temperature_dev = msg.temperature - 273.15;
}


/**
 * Initialize an uavcan interface
 */
void actuators_uavcan_init(struct uavcan_iface_t *iface __attribute__((unused)))
{
  // Check if not already initialized (for multiple interfaces, needs only 1)
  if (actuators_uavcan_initialized) { return; }

  // Bind uavcan ESC_STATUS message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_ESC_STATUS_ID, UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE, &esc_status_ev,
              &actuators_uavcan_esc_status_cb);
  // Bind uavcan ACTUATOR_STATUS message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID, UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE, &actuator_status_ev,
              &actuators_uavcan_actuator_status_cb);
  // Bind uavcan DEVICE_TEMPERATURE message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID, UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE, &device_temperature_ev,
              &actuators_uavcan_device_temperature_cb);

  // Configure telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, actuators_uavcan_send_esc);
#endif

  // Set default to not used
#ifdef SERVOS_UAVCAN1CMD_NB
  for(uint8_t i = 0; i < SERVOS_UAVCAN1CMD_NB; i++)
    actuators_uavcan1cmd_values[i] = UAVCAN_CMD_UNUSED;
#endif
#ifdef SERVOS_UAVCAN2CMD_NB
  for(uint8_t i = 0; i < SERVOS_UAVCAN2CMD_NB; i++)
    actuators_uavcan2cmd_values[i] = UAVCAN_CMD_UNUSED;
#endif

  // Set initialization
  actuators_uavcan_initialized = true;

  // Initialize Random (for telemetry)
  init_random();
}

/**
 * Commit actuator values to the uavcan interface (EQUIPMENT_ESC_RAWCOMMAND)
 */
void actuators_uavcan_commit(struct uavcan_iface_t *iface, int16_t *values, uint8_t nb)
{
  // Generate the message
  struct uavcan_equipment_esc_RawCommand msg = {0};
  msg.cmd.len = nb;
  for (uint8_t i = 0; i < nb; i++) {
    msg.cmd.data[i] = values[i];
  }

  // Encode the mssage
  uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];
  uint32_t total_size = uavcan_equipment_esc_RawCommand_encode(&msg, buffer);

  // Broadcast the raw command message on the interface
  uavcan_broadcast(iface, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID,
                    CANARD_TRANSFER_PRIORITY_HIGH, buffer, total_size);
}

/**
 * Commit actuator values to the uavcan interface (EQUIPMENT_ACTUATOR_ARRAYCOMMAND)
 */
void actuators_uavcan_cmd_commit(struct uavcan_iface_t *iface, int16_t *values, uint8_t nb)
{
  struct uavcan_equipment_actuator_ArrayCommand msg = {0};
  msg.commands.len = 0;

  // Encode the values for each command
  for (uint8_t i = 0; i < nb; i++) {
    // Skip unused commands
    if(values[i] == UAVCAN_CMD_UNUSED || values[i] < MIN_PPRZ || values[i] > MAX_PPRZ)
      continue;
    
    msg.commands.data[msg.commands.len].actuator_id = i;
    msg.commands.data[msg.commands.len].command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS;
    msg.commands.data[msg.commands.len].command_value = (float)values[i] / (float)MAX_PPRZ;
    msg.commands.len++;
  }

  // Encode the message
  uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
  uint32_t total_size = uavcan_equipment_actuator_ArrayCommand_encode(&msg, buffer);

  // Broadcast the raw command message on the interface
  uavcan_broadcast(iface, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID,
                    CANARD_TRANSFER_PRIORITY_HIGH, buffer, total_size);
}
