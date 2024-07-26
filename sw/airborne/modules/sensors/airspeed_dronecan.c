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

/** @file modules/sensors/airspeed_dronecan.c
 * Airspeed sensor on the dronecan bus
 */

#include "airspeed_dronecan.h"
#include "dronecan/dronecan.h"
#include "core/abi.h"

/* Enable ABI sending */
#ifndef AIRSPEED_DRONECAN_SEND_ABI
#define AIRSPEED_DRONECAN_SEND_ABI true
#endif

/* Default pressure scaling */
#ifndef AIRSPEED_DRONECAN_DIFF_P_SCALE
#define AIRSPEED_DRONECAN_DIFF_P_SCALE 1.0f
#endif

/* Airspeed lowpass filter*/
#ifdef USE_AIRSPEED_DRONECAN_LOWPASS_FILTER
#include "filters/low_pass_filter.h"

#ifndef AIRSPEED_DRONECAN_LOWPASS_TAU
#define AIRSPEED_DRONECAN_LOWPASS_TAU 0.15
#endif

#ifndef AIRSPEED_DRONECAN_LOWPASS_PERIOD
#define AIRSPEED_DRONECAN_LOWPASS_PERIOD 0.1
#endif

static Butterworth2LowPass airspeed_filter;
#endif  /* USE_AIRSPEED_DRONECAN_LOWPASS_FILTER */

/* Local variables */
struct airspeed_dronecan_t airspeed_dronecan = {0};
static dronecan_event airspeed_dronecan_ev;

#if FDCAN_PERIPH
static const FDCANExtendedFilter filters[] = {
  {
    0x00040300, // filter RawAirData broadcast
    FILTERING_FEC_FIFO_1,
    0x00FFFF80, // mask
    0,
    FILTERING_FT_MASK // classic filter-mask mode
  }
};
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void airspeed_dronecan_downlink(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t dev_id = DRONECAN_SENDER_ID;
  uint16_t raw = 0;
  float offset = 0;
  float sign = 1.0f;
  if (airspeed_dronecan.diff_p < 0) {
    sign = -1.0f;
  }
  float airspeed = sqrt(airspeed_dronecan.diff_p * sign * 2.0f / 1.225f) * sign;
  pprz_msg_send_AIRSPEED_RAW(trans,dev,AC_ID,
                                &dev_id,
                                &raw,
                                &offset,
                                &airspeed_dronecan.diff_p,
                                &airspeed_dronecan.temperature,
                                &airspeed);
}
#endif /* PERIODIC_TELEMETRY */

static void airspeed_dronecan_cb(struct dronecan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  float diff_p, temperature;
  struct uavcan_equipment_air_data_RawAirData air_data;
  bool decode_error;

  decode_error = uavcan_equipment_air_data_RawAirData_decode(transfer, &air_data);

  if (!decode_error){
    diff_p = air_data.differential_pressure;
    temperature = air_data.static_air_temperature;

    if(!isnan(diff_p)) {
      // Remove the offset and apply a scaling factor
      diff_p -= airspeed_dronecan.diff_p_offset;
      diff_p *= airspeed_dronecan.diff_p_scale;

      // Filtering
#ifdef USE_AIRSPEED_DRONECAN_LOWPASS_FILTER
      float diff_p_filt = update_butterworth_2_low_pass(&airspeed_filter, diff_p);
      airspeed_dronecan.diff_p = diff_p_filt;
#else
      airspeed_dronecan.diff_p = diff_p;
#endif
      // Send the ABI message
#if AIRSPEED_DRONECAN_SEND_ABI
      AbiSendMsgBARO_DIFF(DRONECAN_SENDER_ID, airspeed_dronecan.diff_p);
#endif
    }
    if(!isnan(temperature)) {
      airspeed_dronecan.temperature = temperature;
#if AIRSPEED_DRONECAN_SEND_ABI
      AbiSendMsgTEMPERATURE(DRONECAN_SENDER_ID, airspeed_dronecan.temperature);
#endif
    }
  }
}

void airspeed_dronecan_init(void)
{
  // Set the default values
  airspeed_dronecan.diff_p_scale = AIRSPEED_DRONECAN_DIFF_P_SCALE;

  // Setup the low pass filter
#ifdef USE_AIRSPEED_DRONECAN_LOWPASS_FILTER
  init_butterworth_2_low_pass(&airspeed_filter, AIRSPEED_DRONECAN_LOWPASS_TAU, AIRSPEED_DRONECAN_LOWPASS_PERIOD, 0);
#endif

#if FDCAN_PERIPH
#if DRONECAN_USE_CAN1
  canSTM32SetExtendedFilters(&dronecan1->can_driver, 1, filters);
#endif
#if DRONECAN_USE_CAN2
  canSTM32SetExtendedFilters(&dronecan2->can_driver, 1, filters);
#endif
#endif

  // Bind dronecan RAWAIRDATA message from EQUIPMENT.AIR_DATA
  dronecan_bind(CanardTransferTypeBroadcast,UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE, &airspeed_dronecan_ev, &airspeed_dronecan_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_RAW, airspeed_dronecan_downlink);
#endif
}

void airspeed_dronecan_autoset_offset(bool set) {
  if(set) {
    airspeed_dronecan.diff_p_offset = airspeed_dronecan.diff_p;
  }
}