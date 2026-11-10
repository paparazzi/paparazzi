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

/** @file modules/sensors/airspeed_uavcan.c
 * Airspeed sensor on the uavcan bus
 */

#include "airspeed_uavcan.h"
#include "uavcan/uavcan.h"
#include "core/abi.h"
#include "mcu_periph/sys_time.h"
#include "uavcan.equipment.air_data.RawAirData.h"

/* Enable ABI sending */
#ifndef AIRSPEED_UAVCAN_SEND_ABI
#define AIRSPEED_UAVCAN_SEND_ABI true
#endif

/* Default pressure scaling */
#ifndef AIRSPEED_UAVCAN_DIFF_P_SCALE
#define AIRSPEED_UAVCAN_DIFF_P_SCALE 1.0f
#endif

/* Local variables */
struct airspeed_uavcan_t airspeed_uavcan = {0};
static uavcan_event airspeed_uavcan_ev;

static void airspeed_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  
  struct uavcan_equipment_air_data_RawAirData msg;

  if(uavcan_equipment_air_data_RawAirData_decode(transfer, &msg)) {
    return;   // decode error
  }

#if AIRSPEED_UAVCAN_SEND_ABI
  uint32_t now_ts = get_sys_time_usec();
#endif
  float diff_p = msg.differential_pressure;
  float static_air_temp = msg.static_air_temperature;


  if(!isnan(diff_p)) {
    // Apply a scaling factor, air_data filters and zeroes it and computes the airspeed
    airspeed_uavcan.diff_p = diff_p * airspeed_uavcan.diff_p_scale;

    // Send the ABI message
#if AIRSPEED_UAVCAN_SEND_ABI
    AbiSendMsgBARO_DIFF_RAW(UAVCAN_SENDER_ID, now_ts, airspeed_uavcan.diff_p);
#endif
  }

  if(!isnan(static_air_temp)) {
    airspeed_uavcan.temperature = static_air_temp;
#if AIRSPEED_UAVCAN_SEND_ABI
    AbiSendMsgTEMPERATURE_RAW(UAVCAN_SENDER_ID, now_ts, airspeed_uavcan.temperature, TEMPERATURE_TYPE_AIR);
#endif
  }
}

void airspeed_uavcan_init(void)
{
  // Set the default values
  airspeed_uavcan.diff_p_scale = AIRSPEED_UAVCAN_DIFF_P_SCALE;

  // Bind uavcan RAWAIRDATA message from EQUIPMENT.AIR_DATA
  uavcan_bind(UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE, &airspeed_uavcan_ev, &airspeed_uavcan_cb);
}
