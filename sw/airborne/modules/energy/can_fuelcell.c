/*
 * Copyright (C) 2024 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/energy/can_fuelcell.c"
 * @author F. van Tienen
 * Fuel-cell data through CAN bus
 */

#include "modules/energy/can_fuelcell.h"
#include "modules/datalink/telemetry.h"
#include "uavcan/uavcan.h"
#include "pprz.equipment.fuelcell.Status.h"

/* Fuel-Cell */
struct uavcan_equipment_fuelcell {
  int timeout;

  uint8_t pressure;
  float press_reg;
  float volt_bat;
  float power_out;
  float power_cell;
  float power_batt;
  uint8_t state;
  uint8_t error;
  uint8_t suberror;
};
static struct uavcan_equipment_fuelcell can_fuelcell_data = { 0, 67, 0.78, 45.9, 1457.0, 1234.0, -223.0, 2, 12, 7};

#if !(USE_NPS)

/* CAN feedback */
static uavcan_event fuelcell_uavcan_ev;

static void fuelcell_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{

  struct pprz_equipment_fuelcell_Status msg;
  if(pprz_equipment_fuelcell_Status_decode(transfer, &msg)) {
    return;   // decode error
  }
  
  can_fuelcell_data.pressure = msg.tank_pressure;
  can_fuelcell_data.press_reg = msg.regulated_pressure / 100.0f;
  can_fuelcell_data.volt_bat = msg.battery_voltage / 10.0f;
  can_fuelcell_data.power_out = msg.output_power;
  can_fuelcell_data.power_cell = msg.spm_power;
  can_fuelcell_data.power_batt = msg.battery_power;
  can_fuelcell_data.state = msg.psu_state;
  can_fuelcell_data.error = msg.error_code;
  can_fuelcell_data.suberror = msg.sub_code;

  can_fuelcell_data.timeout = 10;

}
#endif


extern void can_fuelcell_periodic(void)
{
#if USE_NPS
    // Simulate data
    can_fuelcell_data.timeout = 10;
#endif
  if (can_fuelcell_data.timeout > 0) {
    can_fuelcell_data.timeout--;
  }
}



#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void can_fuelcell_send_telemetry(struct transport_tx *trans, struct link_device *dev)
{
  if (can_fuelcell_data.timeout > 0) {
    // Forward
    pprz_msg_send_FUELCELL(trans, dev, AC_ID, &can_fuelcell_data.pressure, &can_fuelcell_data.press_reg,
      &can_fuelcell_data.volt_bat, &can_fuelcell_data.power_out, &can_fuelcell_data.power_cell,
      &can_fuelcell_data.power_batt, &can_fuelcell_data.state, &can_fuelcell_data.error, &can_fuelcell_data.suberror);
  }
}
#endif /* PERIODIC_TELEMETRY */



void can_fuelcell_init(void)
{
#if !(USE_NPS)

    // Bind uavcan BATTERYINFO message from EQUIPMENT.POWER
  uavcan_bind(PPRZ_EQUIPMENT_FUELCELL_STATUS_ID, PPRZ_EQUIPMENT_FUELCELL_STATUS_SIGNATURE, &fuelcell_uavcan_ev,
              &fuelcell_uavcan_cb);
#endif


  // Configure telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FUELCELL, can_fuelcell_send_telemetry);
#endif


}