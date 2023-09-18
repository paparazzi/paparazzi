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
#include "filters/low_pass_filter.h"

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#ifndef AIRSPEED_UAVCAN_LOWPASS_TAU
#define AIRSPEED_UAVCAN_LOWPASS_TAU 0.15
#endif

#ifndef AIRSPEED_UAVCAN_LOWPASS_PERIOD
#define AIRSPEED_UAVCAN_LOWPASS_PERIOD 0.1
#endif

#ifndef AIRSPEED_UAVCAN_SEND_ABI
#define AIRSPEED_UAVCAN_SEND_ABI 1
#endif

#ifdef USE_AIRSPEED_UAVCAN_LOWPASS_FILTER
static Butterworth2LowPass airspeed_filter;
#endif
static uavcan_event airspeed_uavcan_ev;

/* uavcan EQUIPMENT_ESC_STATUS message definition */
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID            1027
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE     (0xC77DF38BA122F5DAULL)
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE      ((397 + 7)/8)

struct airspeed_uavcan_s airspeed_uavcan;

static void airspeed_uavcan_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIRSPEED_UAVCAN(trans,dev,AC_ID,
                                &airspeed_uavcan.diff_p,
                                &airspeed_uavcan.temperature);
}

static void airspeed_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  uint16_t tmp_float = 0;
  float diff_p;

  /* Decode the message */
  //canardDecodeScalar(transfer, (uint32_t)0, 8, false, (void*)&dest->flags);
  //canardDecodeScalar(transfer, (uint32_t)8, 32, false, (void*)&static_p);
  canardDecodeScalar(transfer, (uint32_t)40, 32, false, (void*)&diff_p);
  //canardDecodeScalar(transfer, (uint32_t)72, 16, false, (void*)&tmp_float);
  //float static_temp = canardConvertFloat16ToNativeFloat(tmp_float);
  //canardDecodeScalar(transfer, (uint32_t)88, 16, false, (void*)&tmp_float);
  //float diff_temp = canardConvertFloat16ToNativeFloat(tmp_float);
  canardDecodeScalar(transfer, (uint32_t)104, 16, false, (void*)&tmp_float);
  float static_air_temp = canardConvertFloat16ToNativeFloat(tmp_float);
  //canardDecodeScalar(transfer, (uint32_t)120, 16, false, (void*)&tmp_float);
  //float pitot_temp = canardConvertFloat16ToNativeFloat(tmp_float);

  if(!isnan(diff_p)) {
#ifdef USE_AIRSPEED_UAVCAN_LOWPASS_FILTER
    float diff_p_filt = update_butterworth_2_low_pass(&airspeed_filter, diff_p);
    airspeed_uavcan.diff_p = diff_p_filt;
#else
    airspeed_uavcan.diff_p = diff_p;
#endif

#if AIRSPEED_UAVCAN_SEND_ABI
    AbiSendMsgBARO_DIFF(UAVCAN_SENDER_ID, airspeed_uavcan.diff_p);
#endif
  }

  if(!isnan(static_air_temp)) {
    airspeed_uavcan.temperature = static_air_temp;
#if AIRSPEED_UAVCAN_SEND_ABI
    AbiSendMsgTEMPERATURE(UAVCAN_SENDER_ID, airspeed_uavcan.temperature);
#endif
  }
}

void airspeed_uavcan_init(void)
{
  // Setup low pass filter with time constant and 100Hz sampling freq
#ifdef USE_AIRSPEED_UAVCAN_LOWPASS_FILTER
  init_butterworth_2_low_pass(&airspeed_filter, AIRSPEED_UAVCAN_LOWPASS_TAU, AIRSPEED_UAVCAN_LOWPASS_PERIOD, 0);
#endif

  airspeed_uavcan.diff_p = 0;
  airspeed_uavcan.temperature = 0;

  // Bind uavcan RAWAIRDATA message from EQUIPMENT.AIR_DATA
  uavcan_bind(UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE, &airspeed_uavcan_ev, &airspeed_uavcan_cb);

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_UAVCAN, airspeed_uavcan_downlink);
  #endif
}
