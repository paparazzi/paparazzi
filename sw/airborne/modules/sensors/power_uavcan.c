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

/** @file modules/sensors/power_uavcan.c
 * Power sensors on the uavcan bus
 */

#include "power_uavcan.h"
#include "uavcan/uavcan.h"
#include "modules/energy/electrical.h"
#include "math/pprz_random.h"
#include "uavcan.equipment.power.BatteryInfo.h"
#include "uavcan.equipment.power.CircuitStatus.h"


/* Default maximum amount of batteries */
#ifndef POWER_UAVCAN_BATTERIES_MAX
#define POWER_UAVCAN_BATTERIES_MAX 3
#endif

/* Default maximum amount of circuits */
#ifndef POWER_UAVCAN_CIRCUITS_MAX
#define POWER_UAVCAN_CIRCUITS_MAX 15
#endif

/* Default Battery circuits */
#ifndef POWER_UAVCAN_BATTERY_CIRCUITS
#define POWER_UAVCAN_BATTERY_CIRCUITS {}
#endif

/* Local variables */
static uavcan_event power_uavcan_ev;
static uavcan_event circuit_uavcan_ev;

struct battery_info
{
  bool set;
  uint8_t node_id;
  struct uavcan_equipment_power_BatteryInfo info;
};

/* Circuits */
struct circuit_status {
  bool set;
  uint8_t node_id;
  bool is_battery;
  struct uavcan_equipment_power_CircuitStatus status;
};


static struct battery_info batteries[POWER_UAVCAN_BATTERIES_MAX] = {0};
static struct circuit_status circuits[POWER_UAVCAN_CIRCUITS_MAX] = {0};

/* Battery circuits */
struct uavcan_circuit_battery_t {
  uint8_t node_id;
  uint16_t circuit_id;
};
static struct uavcan_circuit_battery_t battery_circuits[] = POWER_UAVCAN_BATTERY_CIRCUITS;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void power_uavcan_send_power_device(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t idx = 0;
  // Send the circuit status
  if(circuits[idx].set) {
    uint8_t cid = circuits[idx].status.circuit_id;
    pprz_msg_send_POWER_DEVICE(trans, dev, AC_ID, &circuits[idx].node_id, &cid, &circuits[idx].status.current, &circuits[idx].status.voltage);
  }

  // Go to the next
  if (rand_uniform() > 0.02) {
    idx++;
  }
  if(idx >= POWER_UAVCAN_CIRCUITS_MAX || !circuits[idx].set) {
    idx = 0;
  }
}
#endif /* PERIODIC_TELEMETRY */


static void power_uavcan_battery_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  struct uavcan_equipment_power_BatteryInfo msg;
  if(uavcan_equipment_power_BatteryInfo_decode(transfer, &msg)) {
    return;   // decode error
  }

  // Search for the battery or free spot
  uint8_t battery_idx = POWER_UAVCAN_BATTERIES_MAX;
  for (uint8_t i = 0; i < POWER_UAVCAN_BATTERIES_MAX; i++) {
    if (batteries[i].set && batteries[i].node_id == transfer->source_node_id && 
        batteries[i].info.battery_id == msg.battery_id && batteries[i].info.model_instance_id == msg.model_instance_id) {
      battery_idx = i;
      break;
    }
    else if(!batteries[i].set) {
      battery_idx = i;
      break;
    }
  }

  // No free spot found
  if (battery_idx >= POWER_UAVCAN_BATTERIES_MAX) {
    return;
  }

  // Set the battery info
  batteries[battery_idx].set = true;
  batteries[battery_idx].node_id = transfer->source_node_id;
  batteries[battery_idx].info = msg;

  // Sum the battery currents
  float current_sum = 0;
  for (uint8_t i = 0; i < POWER_UAVCAN_BATTERIES_MAX; i++) {
    if (batteries[i].set) {
      current_sum += batteries[i].info.current;
    }
  }
  electrical.current = current_sum;
  if (msg.voltage > 0) {
    electrical.vsupply = msg.voltage;
  }
}

static void power_uavcan_circuit_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{

  struct uavcan_equipment_power_CircuitStatus msg;
  if(uavcan_equipment_power_CircuitStatus_decode(transfer, &msg)) {
    return;   // decode error
  }

  // Search for the circuit or free spot
  uint8_t circuit_idx = POWER_UAVCAN_CIRCUITS_MAX;
  for (uint8_t i = 0; i < POWER_UAVCAN_CIRCUITS_MAX; i++) {
    if (circuits[i].set && circuits[i].node_id == transfer->source_node_id && circuits[i].status.circuit_id == msg.circuit_id) {
      circuit_idx = i;
      break;
    }
    else if(!circuits[i].set) {
      circuit_idx = i;
      break;
    }
  }

  // No free spot found
  if (circuit_idx >= POWER_UAVCAN_CIRCUITS_MAX) {
    return;
  }

  // Set the circuit info
  circuits[circuit_idx].set = true;
  circuits[circuit_idx].node_id = transfer->source_node_id;
  circuits[circuit_idx].status = msg;

  // Sum the 'battery' circuit currents
  float current_sum = 0;
  for (uint8_t i = 0; i < POWER_UAVCAN_CIRCUITS_MAX; i++) {
    if (circuits[i].set && circuits[i].is_battery) {
      current_sum += circuits[i].status.current;
    }
  }
  electrical.current = current_sum;
  if (msg.voltage > 0 && circuits[circuit_idx].is_battery) {
    electrical.vsupply = msg.voltage;
  }
}

void power_uavcan_init(void)
{
  // Init the battery circuits
  for (uint8_t i = 0; i < sizeof(battery_circuits) / sizeof(struct uavcan_circuit_battery_t); i++) {
    circuits[i].set = true;
    circuits[i].node_id = battery_circuits[i].node_id;
    circuits[i].status.circuit_id = battery_circuits[i].circuit_id;
    circuits[i].is_battery = true;
  }

  // Bind uavcan BATTERYINFO message from EQUIPMENT.POWER
  uavcan_bind(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE, &power_uavcan_ev,
              &power_uavcan_battery_cb);

  // Bind uavcan CIRCUIT_STATUS message from EQUIPMENT.POWER
  uavcan_bind(UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ID, UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_SIGNATURE, &circuit_uavcan_ev,
              &power_uavcan_circuit_cb);

  // Configure telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_POWER_DEVICE, power_uavcan_send_power_device);
#endif

  // Initialize Random (for telemetry)
  init_random();
}
