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
};

#ifdef SERVOS_UAVCAN1_NB
int16_t actuators_uavcan1_values[SERVOS_UAVCAN1_NB];
static struct actuators_uavcan_telem_t uavcan1_telem[SERVOS_UAVCAN1_NB] = {0};
#endif
#ifdef SERVOS_UAVCAN2_NB
int16_t actuators_uavcan2_values[SERVOS_UAVCAN2_NB];
static struct actuators_uavcan_telem_t uavcan2_telem[SERVOS_UAVCAN2_NB] = {0};
#endif
#ifdef SERVOS_UAVCAN1CMD_NB
int16_t actuators_uavcan1cmd_values[SERVOS_UAVCAN1CMD_NB];
#endif
#ifdef SERVOS_UAVCAN2CMD_NB
int16_t actuators_uavcan2cmd_values[SERVOS_UAVCAN2CMD_NB];
#endif

/* UNUSED value for CMD */
#define UAVCAN_CMD_UNUSED (MIN_PPRZ-1)

/* uavcan EQUIPMENT_ESC_STATUS message definition */
#define UAVCAN_EQUIPMENT_ESC_STATUS_ID                     1034
#define UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE              (0xA9AF28AEA2FBB254ULL)
#define UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE               ((110 + 7)/8)

/* uavcan EQUIPMENT_ESC_RAWCOMMAND message definition */
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID                 1030
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE          (0x217F5C87D7EC951DULL)
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE           ((285 + 7)/8)

/* uavcan EQUIPMENT_ACTUATOR_STATUS message definition */
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID                1011
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE         (0x5E9BBA44FAF1EA04ULL)
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE          ((64 + 7)/8)

/* uavcan EQUIPMENT_ACTUATOR_ARRAYCOMMAND message definition */
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID          1010
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE   (0xD8A7486238EC3AF3ULL)
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE    ((484 + 7)/8)

/* uavcan EQUIMPENT_DEVICE_TEMPERATURE message definition */
#define UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID             1110
#define UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE      (0x70261C28A94144C6ULL)
#define UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE       ((40 + 7)/8)

/* private variables */
static bool actuators_uavcan_initialized = false;
static uavcan_event esc_status_ev;
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
#ifdef SERVOS_UAVCAN1_NB
  for(uint8_t i = esc_idx - offset; i < SERVOS_UAVCAN1_NB; i++) {
    if(uavcan1_telem[i].set) {
      old_idx = i + offset;
      esc_idx = i + offset + add_idx;
      return &uavcan1_telem[i];
    } else {
      esc_idx = i + offset + 1;
    }
  }
  offset += SERVOS_UAVCAN1_NB;
#endif
#ifdef SERVOS_UAVCAN2_NB
  for(uint8_t i = esc_idx - offset; i < SERVOS_UAVCAN2_NB; i++) {
    if(uavcan2_telem[i].set) {
      old_idx = i + offset;
      esc_idx = i + offset + add_idx;
      return &uavcan2_telem[i];
    } else {
      esc_idx = i + offset + 1;
    }
  }
  offset += SERVOS_UAVCAN2_NB;
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

/**
 * Whevener an ESC_STATUS message from the EQUIPMENT group is received
 */
static void actuators_uavcan_esc_status_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer)
{
  uint8_t esc_idx;
  uint16_t tmp_float;

  struct actuators_uavcan_telem_t *telem = NULL;
  uint8_t max_id = 0;
#ifdef SERVOS_UAVCAN1_NB
  if (iface == &uavcan1) {
    telem = uavcan1_telem;
    max_id = SERVOS_UAVCAN1_NB;
  }
#endif
#ifdef SERVOS_UAVCAN2_NB
  if (iface == &uavcan2) {
    telem = uavcan2_telem;
    max_id = SERVOS_UAVCAN2_NB;
  }
#endif

  canardDecodeScalar(transfer, 105, 5, false, (void *)&esc_idx);
  //Could not find the right interface
  if (esc_idx >= max_id || telem == NULL || max_id == 0) {
    return;
  }
  telem[esc_idx].set = true;
  telem[esc_idx].node_id = transfer->source_node_id;
  telem[esc_idx].timestamp = get_sys_time_float();
  canardDecodeScalar(transfer, 0, 32, false, (void *)&telem[esc_idx].energy);
  canardDecodeScalar(transfer, 32, 16, true, (void *)&tmp_float);
  telem[esc_idx].voltage = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, 48, 16, true, (void *)&tmp_float);
  telem[esc_idx].current = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, 64, 16, true, (void *)&tmp_float);
  telem[esc_idx].temperature = canardConvertFloat16ToNativeFloat(tmp_float) - 273.15;
  canardDecodeScalar(transfer, 80, 18, true, (void *)&telem[esc_idx].rpm);

#if UAVCAN_ACTUATORS_USE_CURRENT
  // Update total current
  electrical.current = 0;
#ifdef SERVOS_UAVCAN1_NB
  for (uint8_t i = 0; i < SERVOS_UAVCAN1_NB; ++i) {
    electrical.current += uavcan1_telem[i].current;
  }
#endif
#ifdef SERVOS_UAVCAN2_NB
  for (uint8_t i = 0; i < SERVOS_UAVCAN2_NB; ++i) {
    electrical.current += uavcan2_telem[i].current;
  }
#endif
#endif
}

/**
 * Whevener an DEVICE_TEMPERATURE message from the EQUIPMENT group is received
 */
static void actuators_uavcan_device_temperature_cb(struct uavcan_iface_t *iface, CanardRxTransfer *transfer)
{
  uint16_t device_id;
  uint16_t tmp_float;

  struct actuators_uavcan_telem_t *telem = NULL;
  uint8_t max_id = 0;
#ifdef SERVOS_UAVCAN1_NB
  if (iface == &uavcan1) {
    telem = uavcan1_telem;
    max_id = SERVOS_UAVCAN1_NB;
  }
#endif
#ifdef SERVOS_UAVCAN2_NB
  if (iface == &uavcan2) {
    telem = uavcan2_telem;
    max_id = SERVOS_UAVCAN2_NB;
  }
#endif


  canardDecodeScalar(transfer, 0, 16, false, (void*)&device_id);
  //Could not find the right interface
  if (device_id >= max_id || telem == NULL || max_id == 0) {
    return;
  }

  canardDecodeScalar(transfer, 16, 16, false, (void*)&tmp_float);
  telem[device_id].temperature_dev = canardConvertFloat16ToNativeFloat(tmp_float) - 273.15;
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
  // Bind uavcan DEVICE_TEMPERATURE message from EQUIPMENT
  uavcan_bind(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID, UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE, &device_temperature_ev,
              &actuators_uavcan_device_temperature_cb);

  // Configure telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, actuators_uavcan_send_esc);
#endif

  // Set default to not set
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
  uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];
  uint32_t offset = 0;

  // Encode the values as 14-bit signed integers
  for (uint8_t i = 0; i < nb; i++) {
    canardEncodeScalar(buffer, offset, 14, (void *)&values[i]);
    offset += 14;
  }

  // Broadcast the raw command message on the interface
  uavcan_broadcast(iface, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID,
                   CANARD_TRANSFER_PRIORITY_HIGH, buffer, (offset + 7) / 8);
}

/**
 * Commit actuator values to the uavcan interface (EQUIPMENT_ACTUATOR_ARRAYCOMMAND)
 */
void actuators_uavcan_cmd_commit(struct uavcan_iface_t *iface, int16_t *values, uint8_t nb)
{
  uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
  uint32_t offset = 0;
  uint8_t command_type = 0; // 0:UNITLESS, 1:meter or radian, 2:N or Nm, 3:m/s or rad/s

  // Encode the values for each command
  for (uint8_t i = 0; i < nb; i++) {
    // Skip unused commands
    if(values[i] == UAVCAN_CMD_UNUSED || values[i] < MIN_PPRZ || values[i] > MAX_PPRZ)
      continue;

    // Set the command id
    canardEncodeScalar(buffer, offset, 8, (void*)&i); // 255
    offset += 8;

    // Set the command type
    canardEncodeScalar(buffer, offset, 8, (void*)&command_type); // 255
    offset += 8;

    // Set the command value
    uint16_t tmp_float = canardConvertNativeFloatToFloat16((float)values[i] / (float)MAX_PPRZ);
    canardEncodeScalar(buffer, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;
  }

  // Broadcast the raw command message on the interface
  uavcan_broadcast(iface, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID,
                   CANARD_TRANSFER_PRIORITY_HIGH, buffer, (offset + 7) / 8);
}
