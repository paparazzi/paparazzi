/*
 * Copyright (C) 2023 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/sensors/power_dronecan.c
 * Power sensors on the dronecan bus
 */

#include "power_dronecan.h"
#include "dronecan/dronecan.h"
#include "modules/energy/electrical.h"

/* Default maximum amount of batteries */
#ifndef POWER_DRONECAN_BATTERIES_MAX
#define POWER_DRONECAN_BATTERIES_MAX 3
#endif

/* Default maximum amount of circuits */
#ifndef POWER_DRONECAN_CIRCUITS_MAX
#define POWER_DRONECAN_CIRCUITS_MAX 15
#endif

/* Default Battery circuits */
#ifndef POWER_DRONECAN_BATTERY_CIRCUITS
#define POWER_DRONECAN_BATTERY_CIRCUITS {}
#endif

/* Local variables */
static dronecan_event power_dronecan_ev;
static dronecan_event circuit_dronecan_ev;

/* Batteries */
struct dronecan_battery_t {
  bool set;
  uint8_t node_id;

  struct uavcan_equipment_power_BatteryInfo battery_info;
};
static struct dronecan_battery_t batteries[POWER_DRONECAN_BATTERIES_MAX] = {0};

/* Circuits */
struct dronecan_circuit_t {
  bool set;
  uint8_t node_id;
  bool is_battery;

  struct uavcan_equipment_power_CircuitStatus circuit_status;
};
static struct dronecan_circuit_t circuits[POWER_DRONECAN_CIRCUITS_MAX] = {0};

/* Battery circuits */
struct dronecan_circuit_battery_t {
  uint8_t node_id;
  uint16_t circuit_id;
};
static struct dronecan_circuit_battery_t battery_circuits[] = POWER_DRONECAN_BATTERY_CIRCUITS;


static void power_dronecan_battery_cb(struct dronecan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  struct uavcan_equipment_power_BatteryInfo battery_info_msg;
  bool decode_error;

  decode_error = uavcan_equipment_power_BatteryInfo_decode(transfer, &battery_info_msg);

  if (!decode_error){
    // Search for the battery or free spot
    uint8_t battery_idx = POWER_DRONECAN_BATTERIES_MAX;
    for (uint8_t i = 0; i < POWER_DRONECAN_BATTERIES_MAX; i++) {
      if (batteries[i].set && batteries[i].node_id == transfer->source_node_id && 
          batteries[i].battery_info.battery_id == battery_info_msg.battery_id && batteries[i].battery_info.model_instance_id == battery_info_msg.model_instance_id) {
        battery_idx = i;
        break;
      }
      else if(!batteries[i].set) {
        battery_idx = i;
        break;
      }
    }

    // No free spot found
    if (battery_idx >= POWER_DRONECAN_BATTERIES_MAX) {
      return;
    }

    // Set the battery info
    batteries[battery_idx].set = true;
    batteries[battery_idx].node_id = transfer->source_node_id;
    batteries[battery_idx].battery_info = battery_info_msg;

    // Sum the battery currents
    float current_sum = 0;
    for (uint8_t i = 0; i < POWER_DRONECAN_BATTERIES_MAX; i++) {
      if (batteries[i].set) {
        current_sum += batteries[i].battery_info.current;
      }
    }
    electrical.current = current_sum;
    if (battery_info_msg.voltage > 0) {
      electrical.vsupply = battery_info_msg.voltage;
    }
  }
}

static void power_dronecan_circuit_cb(struct dronecan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  struct uavcan_equipment_power_CircuitStatus circuit_status_msg;
  bool decode_error;

  decode_error = uavcan_equipment_power_CircuitStatus_decode(transfer, &circuit_status_msg);

  if (!decode_error){
    // Search for the circuit or free spot
    uint8_t circuit_idx = POWER_DRONECAN_CIRCUITS_MAX;
    for (uint8_t i = 0; i < POWER_DRONECAN_CIRCUITS_MAX; i++) {
      if (circuits[i].set && circuits[i].node_id == transfer->source_node_id && circuits[i].circuit_status.circuit_id == circuit_status_msg.circuit_id) {
        circuit_idx = i;
        break;
      }
      else if(!circuits[i].set) {
        circuit_idx = i;
        break;
      }
    }

    // No free spot found
    if (circuit_idx >= POWER_DRONECAN_CIRCUITS_MAX) {
      return;
    }

    // Set the circuit info
    circuits[circuit_idx].set = true;
    circuits[circuit_idx].node_id = transfer->source_node_id;
    circuits[circuit_idx].circuit_status = circuit_status_msg;

    // Sum the 'battery' circuit currents
    float current_sum = 0;
    for (uint8_t i = 0; i < POWER_DRONECAN_CIRCUITS_MAX; i++) {
      if (circuits[i].set && circuits[i].is_battery) {
        current_sum += circuits[i].circuit_status.current;
      }
    }
    electrical.current = current_sum;
    if (circuit_status_msg.voltage > 0 && circuits[circuit_idx].is_battery) {
      electrical.vsupply = circuit_status_msg.voltage;
    }
  }
}

void power_dronecan_init(void)
{
  // Init the battery circuits
  for (uint8_t i = 0; i < sizeof(battery_circuits) / sizeof(struct dronecan_circuit_battery_t); i++) {
    circuits[i].set = true;
    circuits[i].node_id = battery_circuits[i].node_id;
    circuits[i].circuit_status.circuit_id = battery_circuits[i].circuit_id;
    circuits[i].is_battery = true;
  }

  // Bind dronecan BATTERYINFO message from EQUIPMENT.POWER
  dronecan_bind(CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE, &power_dronecan_ev,
              &power_dronecan_battery_cb);

  // Bind dronecan CIRCUIT_STATUS message from EQUIPMENT.POWER
  dronecan_bind(CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ID, UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_SIGNATURE, &circuit_dronecan_ev,
              &power_dronecan_circuit_cb);
}
