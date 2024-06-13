/*
 * Copyright (C) 2024 Paparazzi Team
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
 * @file modules/actuators/actuators_t4.c
 * UAVCan actuators using RAWCOMMAND message and ESC_STATUS telemetry
 * Thanks to AM, FvT and OpenUAS for the initial implementation
 *
 */

#include "modules/energy/electrical.h"
#include "modules/core/abi.h"
#include "modules/actuators/actuators.h"
#include "actuators_t4.h"

/* By default enable the usage of the current sensing in the ESC telemetry */
#ifndef T4_ACTUATORS_USE_CURRENT
#define T4_ACTUATORS_USE_CURRENT TRUE
#endif

/* T4 ESC status telemetry structure */
struct actuators_t4_telem_t {
  bool set;
  uint8_t node_id; //TODO: Servo PCB ID for multiple T4 Boards
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
#ifdef SERVOS_T4X_NB
int16_t actuators_t4x_values[SERVOS_T4X_NB];
#endif

#ifdef SERVOS_T4XCMD_NB
int16_t actuators_t4xcmd_values[SERVOS_T4XCMD_NB];
#endif

#if SERVOS_T4XCMD_NB > SERVOS_T4X_NB
#define T4X_TELEM_NB SERVOS_T4XCMD_NB
static struct actuators_t4_telem_t t4x_telem[SERVOS_T4XCMD_NB] = {0};
#elif defined(SERVOS_T4X_NB)
#define T4X_TELEM_NB SERVOS_T4X_NB
static struct actuators_t4_telem_t t4x_telem[SERVOS_T4X_NB] = {0};
#endif

/* UNUSED value for CMD */
#define T4_CMD_UNUSED (MIN_PPRZ-1)

/* make is Prseudo compatible with UAVCAN values */
/* t4 EQUIPMENT_ESC_STATUS message definition */
#define T4_EQUIPMENT_ESC_STATUS_ID                     1034
#define T4_EQUIPMENT_ESC_STATUS_SIGNATURE              (0xA9AF28AEA2FBB254ULL)
#define T4_EQUIPMENT_ESC_STATUS_MAX_SIZE               ((110 + 7)/8)

/* t4 EQUIPMENT_ESC_RAWCOMMAND message definition */
#define T4_EQUIPMENT_ESC_RAWCOMMAND_ID                 1030
#define T4_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE          (0x217F5C87D7EC951DULL)
#define T4_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE           ((285 + 7)/8)

/* t4 EQUIPMENT_ACTUATOR_STATUS message definition */
#define T4_EQUIPMENT_ACTUATOR_STATUS_ID                1011
#define T4_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE         (0x5E9BBA44FAF1EA04ULL)
#define T4_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE          ((64 + 7)/8)

/* t4 EQUIPMENT_ACTUATOR_ARRAYCOMMAND message definition */
#define T4_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID          1010
#define T4_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE   (0xD8A7486238EC3AF3ULL)
#define T4_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE    ((484 + 7)/8)

/* t4 EQUIMPENT_DEVICE_TEMPERATURE message definition */
#define T4_EQUIPMENT_DEVICE_TEMPERATURE_ID             1110
#define T4_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE      (0x70261C28A94144C6ULL)
#define T4_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE       ((40 + 7)/8)

/* private variables */
static bool actuators_t4_initialized = false;
static t4_event esc_status_ev;
static t4_event actuator_status_ev;
static t4_event device_temperature_ev;

#//if PERIODIC_TELEMETRY
//#include "modules/datalink/telemetry.h" //FIXME: preferably not in here

static uint8_t old_idx = 0;
static uint8_t esc_idx = 0;
static struct actuators_t4_telem_t *actuators_t4_next_telem(void) {
  // Randomness added for multiple  transport devices //TODO: Find a better way to do this
  uint8_t add_idx = 0;
  if (rand_uniform() > 0.02) {
    add_idx = 1;
  }

  // Find the next set telemetry
  uint8_t offset = 0;
#ifdef T4X_TELEM_NB
  for(uint8_t i = esc_idx - offset; i < T4X_TELEM_NB; i++) {
    if(t4x_telem[i].set) {
      old_idx = i + offset;
      esc_idx = i + offset + add_idx;
      return &t4x_telem[i];
    } else {
      esc_idx = i + offset + 1;
    }
  }
  offset += T4X_TELEM_NB;
#endif

  // Going round or no telemetry found
  esc_idx = 0;
  return NULL;
}

static void actuators_t4_send_esc(struct transport_tx *trans, struct link_device *dev)
{
  // Find the correct telemetry (Try twice if going round)
  struct actuators_t4_telem_t *telem = actuators_t4_next_telem();
  if(telem == NULL) {
    telem = actuators_t4_next_telem();
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

//TODO: combine SERVO ESC into one
  // pprz_msg_send_ACTUATOR(trans, dev, AC_ID, &telem->current, &electrical.vsupply, &power,
  //                   &rpm, &telem->voltage, &energy, &telem->temperature, &telem->temperature_dev, &telem->node_id, &old_idx);
}
#endif

/**
 * Whenever an ESC_STATUS message from the EQUIPMENT group is received
 */
static void actuators_t4_esc_status_cb(struct t4_iface_t *iface, T4RxTransfer *transfer)
{
  uint8_t esc_idx;
  uint16_t tmp_float;

  struct actuators_t4_telem_t *telem = NULL;
  uint8_t max_id = 0;
#ifdef T4X_TELEM_NB
  if (iface == &t4x) {
    telem = t4x_telem;
    max_id = T4X_TELEM_NB;
  }
#endif

  //TODO: make a pseudo scaler compatible with SerialBus as in CANard lib
  T4DecodeScalar(transfer, 105, 5, false, (void *)&esc_idx);

  //Could not find the right interface
  if (esc_idx >= max_id || telem == NULL || max_id == 0) {
    return;
  }
  telem[esc_idx].set = true;
  telem[esc_idx].node_id = transfer->source_node_id;
  telem[esc_idx].timestamp = get_sys_time_float();
  t4DecodeScalar(transfer, 0, 32, false, (void *)&telem[esc_idx].energy);
  t4DecodeScalar(transfer, 32, 16, true, (void *)&tmp_float);
  telem[esc_idx].voltage = t4ConvertFloat16ToNativeFloat(tmp_float);
  t4DecodeScalar(transfer, 48, 16, true, (void *)&tmp_float);
  telem[esc_idx].current = t4ConvertFloat16ToNativeFloat(tmp_float);
  t4DecodeScalar(transfer, 64, 16, true, (void *)&tmp_float);
  telem[esc_idx].temperature = T4dConvertFloat16ToNativeFloat(tmp_float) - 273.15;
  t4DecodeScalar(transfer, 80, 18, true, (void *)&telem[esc_idx].rpm);

#if t4_ACTUATORS_USE_CURRENT
  // Update total current
  electrical.current = 0;
#ifdef T4X_TELEM_NB
  for (uint8_t i = 0; i < T4X_TELEM_NB; ++i) {
    electrical.current += t4x_telem[i].current;
  }
#endif

#endif

  // Feedback ABI RPM messages
#ifdef T4X_TELEM_NB
  if (iface == &t4x) {
    struct act_feedback_t feedback = {0};//TODO Refactor all you want EVERYWHERE ;) esc_idx to e.h. actuator_idx
    feedback.rpm = telem[esc_idx].rpm; //
    feedback.set.rpm = true;

#ifdef SERVOS_T4X_NB
    feedback.idx = get_servo_idx_T4X(esc_idx);
#endif
#ifdef SERVOS_T4XCMD_NB
    if(esc_idx < SERVOS_T4XCMD_NB && actuators_t4xcmd_values[esc_idx] != T4_CMD_UNUSED) {
      feedback.idx = get_servo_idx_T4XCMD(esc_idx);
    }
#endif
   //Send the AbI Msg
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_T4_ID, &feedback, 1);
  }
#endif

}

/**
 * Whenever an ACTUATOR_STATUS message from the EQUIPMENT group is received
 */
static void actuators_t4_actuator_status_cb(struct t4_iface_t *iface, T4RxTransfer *transfer)
{
  uint8_t actuator_idx;
  uint16_t tmp_float;

  struct actuators_t4_telem_t *telem = NULL;
  uint8_t max_id = 0;
#ifdef T4X_TELEM_NB
  if (iface == &t4x) {
    telem = t4x_telem;
    max_id = T4X_TELEM_NB;
  }
#endif

  t4DecodeScalar(transfer, 0, 8, false, (void *)&actuator_idx);
  //Could not find the right interface
  if (actuator_idx >= max_id || telem == NULL || max_id == 0) {
    return;
  }

  //telem[actuator_idx].set = true;
  t4DecodeScalar(transfer, 8, 16, true, (void *)&tmp_float);
  telem[actuator_idx].position = t4ConvertFloat16ToNativeFloat(tmp_float);

#ifdef T4X_TELEM_NB
  if (iface == &t4x) {
    struct act_feedback_t feedback = {0};
    feedback.position = telem[actuator_idx].position;
    feedback.set.position = true;

#ifdef SERVOS_T4X_NB
    feedback.idx = get_servo_idx_T4X(actuator_idx);
#endif
#ifdef SERVOS_T4XCMD_NB
    if(actuator_idx < SERVOS_T4XCMD_NB && actuators_t4xcmd_values[actuator_idx] != T4_CMD_UNUSED) {
      feedback.idx = get_servo_idx_T4XCMD(actuator_idx);
    }
#endif

    // Send ABI message
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_T4_ID, &feedback, 1);
  }
#endif

#ifdef T42_TELEM_NB
  if (iface == &t42) {
    struct act_feedback_t feedback = {0};
    feedback.position = telem[actuator_idx].position;
    feedback.set.position = true;

/**
 * Whenever an DEVICE_TEMPERATURE message from the EQUIPMENT group is received
 */
static void actuators_t4_device_temperature_cb(struct t4_iface_t *iface, T4RxTransfer *transfer)
{
  uint16_t device_id;
  uint16_t tmp_float;

  struct actuators_t4_telem_t *telem = NULL;
  uint8_t max_id = 0;
#ifdef T4X_TELEM_NB
  if (iface == &t4x) {
    telem = t4x_telem;
    max_id = T4X_TELEM_NB;
  }
#endif

  t4DecodeScalar(transfer, 0, 16, false, (void*)&device_id);
  //Could not find the right interface
  if (device_id >= max_id || telem == NULL || max_id == 0) {
    return;
  }

  telem[device_id].set = true;
  t4DecodeScalar(transfer, 16, 16, false, (void*)&tmp_float);
  telem[device_id].temperature_dev = t4ConvertFloat16ToNativeFloat(tmp_float) - 273.15;
}


/**
 * Initialize an t4 interface
 */
void actuators_t4_init(struct t4_iface_t *iface __attribute__((unused)))
{
  // Check if not already initialized (for multiple interfaces, needs only 1)
  if (actuators_t4_initialized) { return; }

  // Bind t4 ESC_STATUS message from EQUIPMENT
  t4_bind(T4_EQUIPMENT_ESC_STATUS_ID, T4_EQUIPMENT_ESC_STATUS_SIGNATURE, &esc_status_ev,
              &actuators_t4_esc_status_cb);
  // Bind t4 ACTUATOR_STATUS message from EQUIPMENT
  t4_bind(T4_EQUIPMENT_ACTUATOR_STATUS_ID, T4_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE, &actuator_status_ev,
              &actuators_t4_actuator_status_cb);
  // Bind t4 DEVICE_TEMPERATURE message from EQUIPMENT
  t4_bind(T4_EQUIPMENT_DEVICE_TEMPERATURE_ID, T4_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE, &device_temperature_ev,
              &actuators_t4_device_temperature_cb);

  // Configure telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, actuators_t4_send_esc);
#endif

  // Set default to not set
#ifdef SERVOS_T4XCMD_NB
  for(uint8_t i = 0; i < SERVOS_T4XCMD_NB; i++)
    actuators_t4xcmd_values[i] = T4_CMD_UNUSED;
#endif

  // Set initialization
  actuators_t4_initialized = true;

  // Initialize Random (for telemetry)
  init_random();
}

/**
 * Commit actuator values to the t4 interface (EQUIPMENT_ESC_RAWCOMMAND) //TODO: optional to implement?
 */
void actuators_t4_commit(struct t4_iface_t *iface, int16_t *values, uint8_t nb)
{
  uint8_t buffer[T4_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];
  uint32_t offset = 0;

  // Encode the values as 14-bit signed integers
  for (uint8_t i = 0; i < nb; i++) {
    t4EncodeScalar(buffer, offset, 14, (void *)&values[i]);
    offset += 14;
  }

  // Broadcast the raw command message on the interface
  t4_broadcast(iface, T4_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE, T4_EQUIPMENT_ESC_RAWCOMMAND_ID,
                   T4_TRANSFER_PRIORITY_HIGH, buffer, (offset + 7) / 8);
}

/**
 * Commit actuator values to the t4 interface (EQUIPMENT_ACTUATOR_ARRAYCOMMAND)
 */
void actuators_t4_cmd_commit(struct t4_iface_t *iface, int16_t *values, uint8_t nb)
{
  uint8_t buffer[T4_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
  uint32_t offset = 0;
  uint8_t command_type = 0; // 0:UNITLESS, 1:meter or radian, 2:N or Nm, 3:m/s or rad/s

  // Encode the values for each command
  for (uint8_t i = 0; i < nb; i++) {
    // Skip unused commands
    if(values[i] == T4_CMD_UNUSED || values[i] < MIN_PPRZ || values[i] > MAX_PPRZ)
      continue;

    // Set the command id
    t4EncodeScalar(buffer, offset, 8, (void*)&i); // 255
    offset += 8;

    // Set the command type
    t4EncodeScalar(buffer, offset, 8, (void*)&command_type); // 255
    offset += 8;

    // Set the command value
    uint16_t tmp_float = t4ConvertNativeFloatToFloat16((float)values[i] / (float)MAX_PPRZ);
    t4EncodeScalar(buffer, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;
  }

  // Broadcast the raw command message on the interface
  t4_broadcast(iface, T4_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE, T4_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID,
                   T4_TRANSFER_PRIORITY_HIGH, buffer, (offset + 7) / 8);
}
