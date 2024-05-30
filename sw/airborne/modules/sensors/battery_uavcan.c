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

/** @file modules/sensors/battery_uavcan.c
 * Battery sensor on the uavcan bus
 */

#include "battery_uavcan.h"
#include "uavcan/uavcan.h"
#include "modules/energy/electrical.h"

/* uavcan EQUIPMENT_ESC_STATUS message definition */
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID 1092
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE (0x249C26548A711966ULL)
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE 55

/* Local variables */
static uavcan_event battery_uavcan_ev;

struct uavcan_equipment_power_BatteryInfo {
  bool set;
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
static struct uavcan_equipment_power_BatteryInfo batteries[3];

static void battery_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
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

  // Set the battery info
  if (battery_id < 3) {
    batteries[battery_id].set = true;
    batteries[battery_id].temperature = temperature;
    batteries[battery_id].voltage = voltage;
    batteries[battery_id].current = current;
    batteries[battery_id].average_power_10sec = average_power_10sec;
    batteries[battery_id].remaining_capacity_wh = remaining_capacity_wh;
    batteries[battery_id].full_charge_capacity_wh = full_charge_capacity_wh;
    batteries[battery_id].hours_to_full_charge = hours_to_full_charge;
    batteries[battery_id].status_flags = status_flags;
    batteries[battery_id].state_of_health_pct = state_of_health_pct;
    batteries[battery_id].state_of_charge_pct = state_of_charge_pct;
    batteries[battery_id].state_of_charge_pct_stdev = state_of_charge_pct_stdev;
    batteries[battery_id].battery_id = battery_id;
    batteries[battery_id].model_instance_id = model_instance_id;
  }

  // Sum the battery currents
  float current_sum = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (batteries[i].set) {
      current_sum += batteries[i].current;
    }
  }
  electrical.current = current_sum;
  if (voltage > 0) {
    electrical.vsupply = voltage;
  }
}

void battery_uavcan_init(void)
{
  // Bind uavcan BATTERYINFO message from EQUIPMENT.POWER
  uavcan_bind(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE, &battery_uavcan_ev,
              &battery_uavcan_cb);
}
