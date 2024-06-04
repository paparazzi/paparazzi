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

/* uavcan EQUIPMENT_ESC_STATUS message definition */
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID 1092
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE (0x249C26548A711966ULL)
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE 55

/* uavcan EQUIPMENT_POWER_CIRCUITSTATUS message definition */
#define UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ID 1091
#define UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_SIGNATURE (0x8313D33D0DDDA115ULL)
#define UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_MAX_SIZE 7

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

/* Batteries */
struct uavcan_equipment_power_BatteryInfo {
  bool set;
  uint8_t node_id;

  float temperature;
  float voltage;
  float current;
  float average_power_10sec;
  float remaining_capacity_wh;
  float full_charge_capacity_wh;
  float hours_to_full_charge;
  uint16_t status_flags;
  uint8_t state_of_health_pct;
  uint8_t state_of_charge_pct;
  uint8_t state_of_charge_pct_stdev;
  uint8_t battery_id;
  uint32_t model_instance_id;
//  struct { uint8_t len; uint8_t data[31]; }model_name;
};
static struct uavcan_equipment_power_BatteryInfo batteries[POWER_UAVCAN_BATTERIES_MAX] = {0};

/* Circuits */
struct uavcan_equipment_power_CircuitStatus {
  bool set;
  uint8_t node_id;
  bool is_battery;

  uint16_t circuit_id;
  float voltage;
  float current;
  uint8_t error_flags;
};
static struct uavcan_equipment_power_CircuitStatus circuits[POWER_UAVCAN_CIRCUITS_MAX] = {0};

/* Battery circuits */
struct uavcan_circuit_battery_t {
  uint8_t node_id;
  uint16_t circuit_id;
};
static struct uavcan_circuit_battery_t battery_circuits[] = POWER_UAVCAN_BATTERY_CIRCUITS;


static void power_uavcan_battery_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  uint16_t tmp_float = 0;

  /* Decode the message */
  canardDecodeScalar(transfer, (uint32_t)0, 16, true, (void *)&tmp_float);
  float temperature = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)16, 16, true, (void *)&tmp_float);
  float voltage = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)32, 16, true, (void *)&tmp_float);
  float current = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)48, 16, true, (void *)&tmp_float);
  float average_power_10sec = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)64, 16, true, (void *)&tmp_float);
  float remaining_capacity_wh = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)80, 16, true, (void *)&tmp_float);
  float full_charge_capacity_wh = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)96, 16, true, (void *)&tmp_float);
  float hours_to_full_charge = canardConvertFloat16ToNativeFloat(tmp_float);
  uint16_t status_flags = 0;
  canardDecodeScalar(transfer, (uint32_t)112, 11, false, (void *)&status_flags);
  uint8_t state_of_health_pct = 0;
  canardDecodeScalar(transfer, (uint32_t)123, 7, false, (void *)&state_of_health_pct);
  uint8_t state_of_charge_pct = 0;
  canardDecodeScalar(transfer, (uint32_t)130, 7, false, (void *)&state_of_charge_pct);
  uint8_t state_of_charge_pct_stdev = 0;
  canardDecodeScalar(transfer, (uint32_t)137, 7, false, (void *)&state_of_charge_pct_stdev);
  uint8_t battery_id = 0;
  canardDecodeScalar(transfer, (uint32_t)144, 8, false, (void *)&battery_id);
  uint32_t model_instance_id = 0;
  canardDecodeScalar(transfer, (uint32_t)152, 32, false, (void *)&model_instance_id);

  // Search for the battery or free spot
  uint8_t battery_idx = POWER_UAVCAN_BATTERIES_MAX;
  for (uint8_t i = 0; i < POWER_UAVCAN_BATTERIES_MAX; i++) {
    if (batteries[i].set && batteries[i].node_id == transfer->source_node_id && 
        batteries[i].battery_id == battery_id && batteries[i].model_instance_id == model_instance_id) {
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
  batteries[battery_idx].temperature = temperature;
  batteries[battery_idx].voltage = voltage;
  batteries[battery_idx].current = current;
  batteries[battery_idx].average_power_10sec = average_power_10sec;
  batteries[battery_idx].remaining_capacity_wh = remaining_capacity_wh;
  batteries[battery_idx].full_charge_capacity_wh = full_charge_capacity_wh;
  batteries[battery_idx].hours_to_full_charge = hours_to_full_charge;
  batteries[battery_idx].status_flags = status_flags;
  batteries[battery_idx].state_of_health_pct = state_of_health_pct;
  batteries[battery_idx].state_of_charge_pct = state_of_charge_pct;
  batteries[battery_idx].state_of_charge_pct_stdev = state_of_charge_pct_stdev;
  batteries[battery_idx].battery_id = battery_id;
  batteries[battery_idx].model_instance_id = model_instance_id;

  // Sum the battery currents
  float current_sum = 0;
  for (uint8_t i = 0; i < POWER_UAVCAN_BATTERIES_MAX; i++) {
    if (batteries[i].set) {
      current_sum += batteries[i].current;
    }
  }
  electrical.current = current_sum;
  if (voltage > 0) {
    electrical.vsupply = voltage;
  }
}

static void power_uavcan_circuit_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  uint16_t tmp_float = 0;

  /* Decode the message */
  uint16_t circuit_id = 0;
  canardDecodeScalar(transfer, (uint32_t)0, 16, false, (void *)&circuit_id);
  canardDecodeScalar(transfer, (uint32_t)16, 16, true, (void *)&tmp_float);
  float voltage = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)32, 16, true, (void *)&tmp_float);
  float current = canardConvertFloat16ToNativeFloat(tmp_float);
  uint8_t error_flags = 0;
  canardDecodeScalar(transfer, (uint32_t)48, 8, false, (void *)&error_flags);

  // Search for the circuit or free spot
  uint8_t circuit_idx = POWER_UAVCAN_CIRCUITS_MAX;
  for (uint8_t i = 0; i < POWER_UAVCAN_CIRCUITS_MAX; i++) {
    if (circuits[i].set && circuits[i].node_id == transfer->source_node_id && circuits[i].circuit_id == circuit_id) {
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
  circuits[circuit_idx].circuit_id = circuit_id;
  circuits[circuit_idx].voltage = voltage;
  circuits[circuit_idx].current = current;
  circuits[circuit_idx].error_flags = error_flags;

  // Sum the 'battery' circuit currents
  float current_sum = 0;
  for (uint8_t i = 0; i < POWER_UAVCAN_CIRCUITS_MAX; i++) {
    if (circuits[i].set && circuits[i].is_battery) {
      current_sum += circuits[i].current;
    }
  }
  electrical.current = current_sum;
  if (voltage > 0 && circuits[circuit_idx].is_battery) {
    electrical.vsupply = voltage;
  }
}

void power_uavcan_init(void)
{
  // Init the battery circuits
  for (uint8_t i = 0; i < sizeof(battery_circuits) / sizeof(struct uavcan_circuit_battery_t); i++) {
    circuits[i].set = true;
    circuits[i].node_id = battery_circuits[i].node_id;
    circuits[i].circuit_id = battery_circuits[i].circuit_id;
    circuits[i].is_battery = true;
  }

  // Bind uavcan BATTERYINFO message from EQUIPMENT.POWER
  uavcan_bind(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE, &power_uavcan_ev,
              &power_uavcan_battery_cb);

  // Bind uavcan CIRCUIT_STATUS message from EQUIPMENT.POWER
  uavcan_bind(UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ID, UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_SIGNATURE, &circuit_uavcan_ev,
              &power_uavcan_circuit_cb);
}
