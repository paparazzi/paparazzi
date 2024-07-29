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
 * @file modules/actuators/actuators_dronecan.c
 * DroneCAN actuators using RAWCOMMAND message and ESC_STATUS telemetry
 *
 */

#include "actuators_dronecan.h"
#include "modules/energy/electrical.h"
#include "math/pprz_random.h"
#include "modules/core/abi.h"
#include "modules/actuators/actuators.h"

/* By default enable the usage of the current sensing in the ESC telemetry */
#ifndef DRONECAN_ACTUATORS_USE_CURRENT
#define DRONECAN_ACTUATORS_USE_CURRENT TRUE
#endif

/* dronecan ESC status telemetry structure */
struct actuators_dronecan_telem_t {
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
#ifdef SERVOS_DRONECAN1_NB
int16_t actuators_dronecan1_values[SERVOS_DRONECAN1_NB];
#endif
#ifdef SERVOS_DRONECAN2_NB
int16_t actuators_dronecan2_values[SERVOS_DRONECAN2_NB];
#endif
#ifdef SERVOS_DRONECAN1CMD_NB
int16_t actuators_dronecan1cmd_values[SERVOS_DRONECAN1CMD_NB];
#endif
#ifdef SERVOS_DRONECAN2CMD_NB
int16_t actuators_dronecan2cmd_values[SERVOS_DRONECAN2CMD_NB];
#endif

/* Set the actual telemetry length (ID's from actuators can't collide with the command version) */
#if SERVOS_DRONECAN1CMD_NB > SERVOS_DRONECAN1_NB
#define DRONECAN1_TELEM_NB SERVOS_DRONECAN1CMD_NB
static struct actuators_dronecan_telem_t dronecan1_telem[SERVOS_DRONECAN1CMD_NB] = {0};
#elif defined(SERVOS_DRONECAN1_NB)
#define DRONECAN1_TELEM_NB SERVOS_DRONECAN1_NB
static struct actuators_dronecan_telem_t dronecan1_telem[SERVOS_DRONECAN1_NB] = {0};
#endif

#if SERVOS_DRONECAN2CMD_NB > SERVOS_DRONECAN2_NB
#define DRONECAN2_TELEM_NB SERVOS_DRONECAN2CMD_NB
static struct actuators_dronecan_telem_t dronecan2_telem[SERVOS_DRONECAN2CMD_NB] = {0};
#elif defined(SERVOS_DRONECAN2_NB)
#define DRONECAN2_TELEM_NB SERVOS_DRONECAN2_NB
static struct actuators_dronecan_telem_t dronecan2_telem[SERVOS_DRONECAN2_NB] = {0};
#endif

/* UNUSED value for CMD */
#define DRONECAN_CMD_UNUSED (MIN_PPRZ-1)

/* private variables */
static bool actuators_dronecan_initialized = false;
static dronecan_event esc_status_ev;
static dronecan_event actuator_status_ev;
static dronecan_event device_temperature_ev;

// #if FDCAN_PERIPH
// static const FDCANExtendedFilter filters[] = {
//   {
//     0x00040A00, // filter ESCStatus broadcast
//     FILTERING_FEC_FIFO_0,
//     0x00FFFF80, // mask
//     0,
//     FILTERING_FT_MASK // classic filter-mask mode
//   },
//   {
//     0x0003F300, // filter ActuatorStatus broadcast
//     FILTERING_FEC_FIFO_0,
//     0x00FFFF80, // mask
//     0,
//     FILTERING_FT_MASK // classic filter-mask mode
//   },
//   {
//     0x00045600, // filter DeviceTemperature broadcast
//     FILTERING_FEC_FIFO_0,
//     0x00FFFF80, // mask
//     0,
//     FILTERING_FT_MASK // classic filter-mask mode
//   }
// };
// #endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static uint8_t old_idx = 0;
static uint8_t esc_idx = 0;
static struct actuators_dronecan_telem_t *actuators_dronecan_next_telem(void) {
  // Randomness added for multiple transport devices
  uint8_t add_idx = 0;
  if (rand_uniform() > 0.02) {
    add_idx = 1;
  }

  // Find the next set telemetry
  uint8_t offset = 0;
#ifdef DRONECAN1_TELEM_NB
  for(uint8_t i = esc_idx - offset; i < DRONECAN1_TELEM_NB; i++) {
    if(dronecan1_telem[i].set) {
      old_idx = i + offset;
      esc_idx = i + offset + add_idx;
      return &dronecan1_telem[i];
    } else {
      esc_idx = i + offset + 1;
    }
  }
  offset += DRONECAN1_TELEM_NB;
#endif
#ifdef DRONECAN2_TELEM_NB
  for(uint8_t i = esc_idx - offset; i < DRONECAN2_TELEM_NB; i++) {
    if(dronecan2_telem[i].set) {
      old_idx = i + offset;
      esc_idx = i + offset + add_idx;
      return &dronecan2_telem[i];
    } else {
      esc_idx = i + offset + 1;
    }
  }
  offset += DRONECAN2_TELEM_NB;
#endif

  // Going round or no telemetry found
  esc_idx = 0;
  return NULL;
}

static void actuators_dronecan_send_esc(struct transport_tx *trans, struct link_device *dev)
{
  // Find the correct telemetry (Try twice if going round)
  struct actuators_dronecan_telem_t *telem = actuators_dronecan_next_telem();
  if(telem == NULL) {
    telem = actuators_dronecan_next_telem();
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

/**
 * Whevener an ESC_STATUS message from the EQUIPMENT group is received
 */
static void actuators_dronecan_esc_status_cb(struct dronecan_iface_t *iface, CanardRxTransfer *transfer)
{
  uint8_t esc_idx;
  struct actuators_dronecan_telem_t *telem = NULL;
  struct uavcan_equipment_esc_Status status;
  bool decode_error;
  uint8_t max_id = 0;
#ifdef DRONECAN1_TELEM_NB
  if (iface == &dronecan1) {
    telem = dronecan1_telem;
    max_id = DRONECAN1_TELEM_NB;
  }
#endif
#ifdef DRONECAN2_TELEM_NB
  if (iface == &dronecan2) {
    telem = dronecan2_telem;
    max_id = DRONECAN2_TELEM_NB;
  }
#endif
  decode_error = uavcan_equipment_esc_Status_decode(transfer, &status);
  if (!decode_error){
    esc_idx = status.esc_index;
    //Could not find the right interface
    if (esc_idx >= max_id || telem == NULL) {
      return;
    }
    telem[esc_idx].set = true;
    telem[esc_idx].node_id = transfer->source_node_id;
    telem[esc_idx].timestamp = get_sys_time_float();
    telem[esc_idx].energy = status.error_count; // If the field really was energy, it changed in the new dronecan version ?
    telem[esc_idx].voltage = status.voltage;
    telem[esc_idx].current = status.current;
    telem[esc_idx].temperature = status.temperature - 273.15; // K -> °C conversion
    telem[esc_idx].rpm = status.rpm;
  }


#if DRONECAN_ACTUATORS_USE_CURRENT
  // Update total current
  electrical.current = 0;
#ifdef DRONECAN1_TELEM_NB
  for (uint8_t i = 0; i < DRONECAN1_TELEM_NB; ++i) {
    electrical.current += dronecan1_telem[i].current;
  }
#endif
#ifdef DRONECAN2_TELEM_NB
  for (uint8_t i = 0; i < DRONECAN2_TELEM_NB; ++i) {
    electrical.current += dronecan2_telem[i].current;
  }
#endif
#endif

  // Feedback ABI RPM messages
#ifdef DRONECAN1_TELEM_NB
  if (iface == &dronecan1) {
    struct act_feedback_t feedback = {0};
    feedback.rpm = telem[esc_idx].rpm;
    feedback.set.rpm = true;

#ifdef SERVOS_DRONECAN1_NB
    feedback.idx = get_servo_idx_DRONECAN1(esc_idx);
#endif
#ifdef SERVOS_DRONECAN1CMD_NB
    if(esc_idx < SERVOS_DRONECAN1CMD_NB && actuators_dronecan1cmd_values[esc_idx] != DRONECAN_CMD_UNUSED) {
      feedback.idx = get_servo_idx_DRONECAN1CMD(esc_idx);
    }
#endif

    // Send ABI message
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_DRONECAN_ID, &feedback, 1);
  }
#endif
#ifdef DRONECAN2_TELEM_NB
  if (iface == &dronecan2) {
    struct act_feedback_t feedback = {0};
    feedback.rpm = telem[esc_idx].rpm;
    feedback.set.rpm = true;

#ifdef SERVOS_DRONECAN2_NB
    feedback.idx = get_servo_idx_DRONECAN2(esc_idx);
#endif
#ifdef SERVOS_DRONECAN2CMD_NB
    if(esc_idx < SERVOS_DRONECAN2CMD_NB && actuators_dronecan2cmd_values[esc_idx] != DRONECAN_CMD_UNUSED) {
      feedback.idx = get_servo_idx_DRONECAN2CMD(esc_idx);
    }
#endif

    // Send ABI message
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_DRONECAN_ID, &feedback, 1);
  }
#endif
}

/**
 * Whevener an ACTUATOR_STATUS message from the EQUIPMENT group is received
 */
static void actuators_dronecan_actuator_status_cb(struct dronecan_iface_t *iface, CanardRxTransfer *transfer)
{
  uint8_t actuator_idx;
  struct actuators_dronecan_telem_t *telem = NULL;
  struct uavcan_equipment_actuator_Status status;
  bool decode_error;
  uint8_t max_id = 0;
#ifdef DRONECAN1_TELEM_NB
  if (iface == &dronecan1) {
    telem = dronecan1_telem;
    max_id = DRONECAN1_TELEM_NB;
  }
#endif
#ifdef DRONECAN2_TELEM_NB
  if (iface == &dronecan2) {
    telem = dronecan2_telem;
    max_id = DRONECAN2_TELEM_NB;
  }
#endif
  decode_error = uavcan_equipment_actuator_Status_decode(transfer,&status);
  if (!decode_error){
    actuator_idx = status.actuator_id;
    //Could not find the right interface
    if (actuator_idx >= max_id || telem == NULL) {
      return;
    }
    telem[actuator_idx].set = true;
    telem[actuator_idx].position = status.position;
  }

#ifdef DRONECAN1_TELEM_NB
  if (iface == &dronecan1) {
    struct act_feedback_t feedback = {0};
    feedback.position = telem[actuator_idx].position;
    feedback.set.position = true;

#ifdef SERVOS_DRONECAN1_NB
    feedback.idx = get_servo_idx_DRONECAN1(actuator_idx);
#endif
#ifdef SERVOS_DRONECAN1CMD_NB
    if(actuator_idx < SERVOS_DRONECAN1CMD_NB && actuators_dronecan1cmd_values[actuator_idx] != DRONECAN_CMD_UNUSED) {
      feedback.idx = get_servo_idx_DRONECAN1CMD(actuator_idx);
    }
#endif

    // Send ABI message
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_DRONECAN_ID, &feedback, 1);
  }
#endif

#ifdef DRONECAN2_TELEM_NB
  if (iface == &dronecan2) {
    struct act_feedback_t feedback = {0};
    feedback.position = telem[actuator_idx].position;
    feedback.set.position = true;

#ifdef SERVOS_DRONECAN2_NB
    feedback.idx = get_servo_idx_DRONECAN2(actuator_idx);
#endif
#ifdef SERVOS_DRONECAN2CMD_NB
    if(actuator_idx < SERVOS_DRONECAN2CMD_NB && actuators_dronecan2cmd_values[actuator_idx] != DRONECAN_CMD_UNUSED) {
      feedback.idx = get_servo_idx_DRONECAN2CMD(actuator_idx);
    }
#endif

    // Send ABI message
    AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_DRONECAN_ID, &feedback, 1);
  }
#endif
}

/**
 * Whevener an DEVICE_TEMPERATURE message from the EQUIPMENT group is received
 */
static void actuators_dronecan_device_temperature_cb(struct dronecan_iface_t *iface, CanardRxTransfer *transfer)
{
  uint16_t device_id;
  struct actuators_dronecan_telem_t *telem = NULL;
  struct uavcan_equipment_device_Temperature status;
  bool decode_error;
  uint8_t max_id = 0;
#ifdef DRONECAN1_TELEM_NB
  if (iface == &dronecan1) {
    telem = dronecan1_telem;
    max_id = DRONECAN1_TELEM_NB;
  }
#endif
#ifdef DRONECAN2_TELEM_NB
  if (iface == &dronecan2) {
    telem = dronecan2_telem;
    max_id = DRONECAN2_TELEM_NB;
  }
#endif

  decode_error = uavcan_equipment_device_Temperature_decode(transfer,&status);
  if (!decode_error){
    device_id = status.device_id;
    //Could not find the right interface
    if (device_id >= max_id || telem == NULL) {
      return;
    }
    telem[device_id].set = true;
    telem[device_id].temperature_dev = status.temperature - 273.15; // K -> °C conversion
  }
}


/**
 * Initialize a dronecan interface
 */
void actuators_dronecan_init(struct dronecan_iface_t *iface __attribute__((unused)))
{
  // Check if not already initialized (for multiple interfaces, needs only 1)
  if (actuators_dronecan_initialized) { return; }

// #if FDCAN_PERIPH
// #if DRONECAN_USE_CAN1
//   canSTM32SetExtendedFilters(&dronecan1->can_driver, 3, filters);
// #endif
// #if DRONECAN_USE_CAN2
//   canSTM32SetExtendedFilters(&dronecan2->can_driver, 3, filters);
// #endif
// #endif

  // Bind dronecan ESC_STATUS message from EQUIPMENT
  dronecan_bind(CanardTransferTypeBroadcast,UAVCAN_EQUIPMENT_ESC_STATUS_ID, UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE, &esc_status_ev,
              &actuators_dronecan_esc_status_cb);
  // Bind dronecan ACTUATOR_STATUS message from EQUIPMENT
  dronecan_bind(CanardTransferTypeBroadcast,UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID, UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE, &actuator_status_ev,
              &actuators_dronecan_actuator_status_cb);
  // Bind dronecan DEVICE_TEMPERATURE message from EQUIPMENT
  dronecan_bind(CanardTransferTypeBroadcast,UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID, UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE, &device_temperature_ev,
              &actuators_dronecan_device_temperature_cb);

  // Configure telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, actuators_dronecan_send_esc);
#endif

  // Set default to not set
#ifdef SERVOS_DRONECAN1CMD_NB
  for(uint8_t i = 0; i < SERVOS_DRONECAN1CMD_NB; i++)
    actuators_dronecan1cmd_values[i] = DRONECAN_CMD_UNUSED;
#endif
#ifdef SERVOS_DRONECAN2CMD_NB
  for(uint8_t i = 0; i < SERVOS_DRONECAN2CMD_NB; i++)
    actuators_dronecan2cmd_values[i] = DRONECAN_CMD_UNUSED;
#endif

  // Set initialization
  actuators_dronecan_initialized = true;

  // Initialize Random (for telemetry)
  init_random();
}

/**
 * Commit actuator values to the dronecan interface (EQUIPMENT_ESC_RAWCOMMAND)
 */
void actuators_dronecan_commit(struct dronecan_iface_t *iface, int16_t *values, uint8_t nb)
{
  uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];
  struct uavcan_equipment_esc_RawCommand command;
  command.cmd.len = nb; 
  memcpy(command.cmd.data,values,sizeof(values));

  uint32_t len = uavcan_equipment_esc_RawCommand_encode(&command, buffer
#if FDCAN_ENABLED
    , 0
#endif
  );

  static uint8_t transfer_id;
  static CanardTxTransfer broadcast;
  canardInitTxTransfer(&broadcast);

  broadcast.transfer_type = CanardTransferTypeBroadcast;
  broadcast.data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
  broadcast.data_type_id = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID;
  broadcast.inout_transfer_id = &transfer_id;
  broadcast.priority = CANARD_TRANSFER_PRIORITY_LOW;
  broadcast.payload = buffer;
  broadcast.payload_len = len;
#if FDCAN_ENABLED
  broadcast.canfd = 1;
  broadcast.tao = 0;
#endif

  // Broadcast the raw command message on the interface
  dronecan_broadcast(iface, &broadcast);
}

/**
 * Commit actuator values to the dronecan interface (EQUIPMENT_ACTUATOR_ARRAYCOMMAND)
 */
void actuators_dronecan_cmd_commit(struct dronecan_iface_t *iface, int16_t *values, uint8_t nb)
{
  uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
  struct uavcan_equipment_actuator_ArrayCommand array;
  struct uavcan_equipment_actuator_Command cmds[nb];
  uint8_t cmd_type = 0; // 0:UNITLESS, 1:meter or radian, 2:N or Nm, 3:m/s or rad/s
  for (uint8_t i = 0; i < nb ; i++){
    cmds[i].actuator_id = i;
    cmds[i].command_type = cmd_type;
    cmds[i].command_value = canardConvertFloat16ToNativeFloat(values[i]);
  }
  array.commands.len = nb;
  memcpy(array.commands.data,cmds,sizeof(cmds));

  uint32_t len = uavcan_equipment_actuator_ArrayCommand_encode(&array, buffer
#if FDCAN_ENABLED
    , 0
#endif
  );

  static uint8_t transfer_id;
  static CanardTxTransfer broadcast;
  canardInitTxTransfer(&broadcast);

  broadcast.transfer_type = CanardTransferTypeBroadcast;
  broadcast.data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
  broadcast.data_type_id = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID;
  broadcast.inout_transfer_id = &transfer_id;
  broadcast.priority = CANARD_TRANSFER_PRIORITY_LOW;
  broadcast.payload = buffer;
  broadcast.payload_len = len;
#if FDCAN_ENABLED
  broadcast.canfd = 1;
  broadcast.tao = 0;
#endif
 
  // Broadcast the raw command message on the interface
  dronecan_broadcast(iface, &broadcast);
}