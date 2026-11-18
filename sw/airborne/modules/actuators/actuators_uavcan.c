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
#include "modules/core/abi.h"
#include "modules/actuators/actuators.h"
#include <math.h>


#include "uavcan.equipment.esc.Status.h"
#include "uavcan.equipment.esc.StatusExtended.h"
#include "uavcan.equipment.esc.RawCommand.h"
#include "uavcan.equipment.actuator.Status.h"
#include "uavcan.equipment.actuator.ArrayCommand.h"
#include "uavcan.equipment.device.Temperature.h"

/* By default enable the usage of the current sensing in the ESC telemetry */
#ifndef UAVCAN_ACTUATORS_USE_CURRENT
#define UAVCAN_ACTUATORS_USE_CURRENT TRUE
#endif

#define UNITLESS    UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS
#define POSITION    UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION
#define FORCE       UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_FORCE
#define SPEED       UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_SPEED
#define PWM         UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM

/* uavcan ESC status telemetry structure */
struct actuators_uavcan_telem_t {
  bool set;
  float voltage;
  float current;
  float temperature;
  int32_t rpm;
  float position;
};

/* The transmitted actuator values */
#ifdef SERVOS_UAVCAN1_NB
int16_t actuators_uavcan1_values[SERVOS_UAVCAN1_NB];

void actuators_uavcan1_set(uint8_t idx, int16_t value)
{
  actuators_uavcan1_values[idx] = value;
}
#endif
#ifdef SERVOS_UAVCAN2_NB
int16_t actuators_uavcan2_values[SERVOS_UAVCAN2_NB];

void actuators_uavcan2_set(uint8_t idx, int16_t value)
{
  actuators_uavcan2_values[idx] = value;
}
#endif

#ifdef SERVOS_UAVCAN1CMD_NB

#ifndef ACTUATOR_UAVCAN1_CMD_TYPES
#define ACTUATOR_UAVCAN1_CMD_TYPES {[0 ... SERVOS_UAVCAN1CMD_NB - 1] = UNITLESS}
#endif

#ifndef ACTUATOR_UAVCAN1_CMD_FACTORS
#define ACTUATOR_UAVCAN1_CMD_FACTORS {[0 ... SERVOS_UAVCAN1CMD_NB - 1] = (float)MAX_PPRZ}
#endif

static const uint8_t uavcan1_cmd_types[SERVOS_UAVCAN1CMD_NB] = ACTUATOR_UAVCAN1_CMD_TYPES;
static const float uavcan1_cmd_factors[SERVOS_UAVCAN1CMD_NB] = ACTUATOR_UAVCAN1_CMD_FACTORS;

int16_t actuators_uavcan1cmd_values[SERVOS_UAVCAN1CMD_NB];

void actuators_uavcan1cmd_set(uint8_t idx, int16_t value)
{
  actuators_uavcan1cmd_values[idx] = value;
}

#endif

#ifdef SERVOS_UAVCAN2CMD_NB
#ifndef ACTUATOR_UAVCAN2_CMD_TYPES
#define ACTUATOR_UAVCAN2_CMD_TYPES {[0 ... SERVOS_UAVCAN2CMD_NB - 1] = UNITLESS}
#endif

#ifndef ACTUATOR_UAVCAN2_CMD_FACTORS
#define ACTUATOR_UAVCAN2_CMD_FACTORS {[0 ... SERVOS_UAVCAN2CMD_NB - 1] = (float)MAX_PPRZ}
#endif

static const uint8_t uavcan2_cmd_types[SERVOS_UAVCAN2CMD_NB] = ACTUATOR_UAVCAN2_CMD_TYPES;
static const float uavcan2_cmd_factors[SERVOS_UAVCAN2CMD_NB] = ACTUATOR_UAVCAN2_CMD_FACTORS;

int16_t actuators_uavcan2cmd_values[SERVOS_UAVCAN2CMD_NB];

void actuators_uavcan2cmd_set(uint8_t idx, int16_t value)
{
  actuators_uavcan2cmd_values[idx] = value;
}
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
static uavcan_event esc_status_extended_ev;
static uavcan_event actuator_status_ev;
static uavcan_event device_temperature_ev;

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
    // First try as RAW COMMAND (the generated lookup gives 0 for an unknown index, so check the actuator)
    if(idx < SERVOS_UAVCAN1_NB) {
      uint8_t raw_idx = get_servo_idx_UAVCAN1(idx);
      if(raw_idx < ACTUATORS_NB && actuators[raw_idx].set == actuators_uavcan1_set && actuators[raw_idx].config.servo_idx == idx) {
        actuator_idx = raw_idx;
      }
    }
#endif
#ifdef SERVOS_UAVCAN1CMD_NB
    // Then try as ACTUATOR COMMAND (also before the first command is sent)
    if(idx < SERVOS_UAVCAN1CMD_NB) {
      uint8_t cmd_idx = get_servo_idx_UAVCAN1CMD(idx);
      if(cmd_idx < ACTUATORS_NB && actuators[cmd_idx].set == actuators_uavcan1cmd_set && actuators[cmd_idx].config.servo_idx == idx) {
        actuator_idx = cmd_idx;
      }
    }
#endif
  }
#endif
#ifdef UAVCAN2_TELEM_NB
  if(iface == &uavcan2) {
#ifdef SERVOS_UAVCAN2_NB
    // First try as RAW COMMAND (the generated lookup gives 0 for an unknown index, so check the actuator)
    if(idx < SERVOS_UAVCAN2_NB) {
      uint8_t raw_idx = get_servo_idx_UAVCAN2(idx);
      if(raw_idx < ACTUATORS_NB && actuators[raw_idx].set == actuators_uavcan2_set && actuators[raw_idx].config.servo_idx == idx) {
        actuator_idx = raw_idx;
      }
    }
#endif
#ifdef SERVOS_UAVCAN2CMD_NB
    // Then try as ACTUATOR COMMAND (also before the first command is sent)
    if(idx < SERVOS_UAVCAN2CMD_NB) {
      uint8_t cmd_idx = get_servo_idx_UAVCAN2CMD(idx);
      if(cmd_idx < ACTUATORS_NB && actuators[cmd_idx].set == actuators_uavcan2cmd_set && actuators[cmd_idx].config.servo_idx == idx) {
        actuator_idx = cmd_idx;
      }
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
  telem->voltage = msg.voltage;
  telem->current = msg.current;
  telem->temperature = msg.temperature;
  telem->rpm = msg.rpm;

  /* Specification says Kelvin, but some are transmitting in Celsius */
  if (telem->temperature > 230.f) {
    telem->temperature -= 273.15;
  }

  // Feedback ABI messages
  struct act_feedback_t feedback = {0};
  feedback.idx = get_actuator_idx(iface, msg.esc_index);
  feedback.rpm = telem->rpm;
  feedback.set.rpm = true;
  feedback.current = telem->current;
  feedback.set.current = !isnan(telem->current);
  feedback.voltage = telem->voltage;
  feedback.set.voltage = !isnan(telem->voltage);
  feedback.temperature = telem->temperature;
  feedback.set.temperature = !isnan(telem->temperature);
  feedback.errors = msg.error_count;
  feedback.set.errors = true;

  // Send ABI message
  if (feedback.idx < ACTUATORS_NB) {
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
  }

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
  // Unknown fields are NAN, unknown power rating is 127
  feedback.position = telem->position;
  feedback.set.position = !isnan(msg.position);
  feedback.force = msg.force;
  feedback.set.force = !isnan(msg.force);
  feedback.speed = msg.speed;
  feedback.set.speed = !isnan(msg.speed);
  if (msg.power_rating_pct <= 100) {
    feedback.power_pct = msg.power_rating_pct;
    feedback.set.power = true;
  }

  // Send ABI message
  if (feedback.idx < ACTUATORS_NB) {
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
  }
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

  // Feedback ABI message (e.g. the temperature of a servo)
  struct act_feedback_t feedback = {0};
  feedback.idx = get_actuator_idx(iface, msg.device_id);
  feedback.temperature = msg.temperature - 273.15f;
  feedback.set.temperature = !isnan(msg.temperature);
  if (feedback.idx < ACTUATORS_NB) {
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
  }
}


/**
 * Whevener an ESC_STATUS_EXTENDED message from the EQUIPMENT group is received
 */
static void actuators_uavcan_esc_status_extended_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer)
{
  struct uavcan_equipment_esc_StatusExtended msg;
  if (uavcan_equipment_esc_StatusExtended_decode(transfer, &msg)) {
    return; // Decoding error
  }

  struct act_feedback_t feedback = {0};
  feedback.idx = get_actuator_idx(iface, msg.esc_index);
  feedback.output_pct = msg.output_pct;
  feedback.set.output = true;
  feedback.motor_temperature = msg.motor_temperature_degC;
  feedback.set.motor_temperature = true;
  if (feedback.idx < ACTUATORS_NB) {
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_UAVCAN_ID, &feedback, 1);
  }
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
  // Bind uavcan ESC_STATUS_EXTENDED message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_ID, UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_SIGNATURE, &esc_status_extended_ev,
              &actuators_uavcan_esc_status_extended_cb);
  // Bind uavcan ACTUATOR_STATUS message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID, UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE, &actuator_status_ev,
              &actuators_uavcan_actuator_status_cb);
  // Bind uavcan DEVICE_TEMPERATURE message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID, UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE, &device_temperature_ev,
              &actuators_uavcan_device_temperature_cb);


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

  const uint8_t *cmd_types = NULL;
  const float *cmd_factors = NULL;

#ifdef SERVOS_UAVCAN1CMD_NB
  if(iface == &uavcan1) {
    cmd_types = uavcan1_cmd_types;
    cmd_factors = uavcan1_cmd_factors;
  }
#endif
#ifdef SERVOS_UAVCAN2CMD_NB
  if(iface == &uavcan2) {
    cmd_types = uavcan2_cmd_types;
    cmd_factors = uavcan2_cmd_factors;
  }
#endif

  if (cmd_types == NULL || cmd_factors == NULL) {
    return;
  }

  struct uavcan_equipment_actuator_ArrayCommand msg = {0};
  msg.commands.len = 0;

  // Encode the values for each command
  for (uint8_t i = 0; i < nb; i++) {
    // Skip unused commands
    if(values[i] == UAVCAN_CMD_UNUSED || values[i] < MIN_PPRZ || values[i] > MAX_PPRZ)
      continue;
    
    msg.commands.data[msg.commands.len].actuator_id = i;
    msg.commands.data[msg.commands.len].command_type = cmd_types[i];
    msg.commands.data[msg.commands.len].command_value = (float)values[i] / cmd_factors[i];
    msg.commands.len++;
  }

  // Nothing to send (e.g. actuators not commanded yet)
  if (msg.commands.len == 0) {
    return;
  }

  // Encode the message
  uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
  uint32_t total_size = uavcan_equipment_actuator_ArrayCommand_encode(&msg, buffer);

  // Broadcast the raw command message on the interface
  uavcan_broadcast(iface, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID,
                    CANARD_TRANSFER_PRIORITY_HIGH, buffer, total_size);
}
