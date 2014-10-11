/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file subsystems/air_data.c
 * Air Data interface
 *  - pressures
 *  - airspeed
 *  - angle of attack and sideslip
 *  - wind
 */

#include "subsystems/air_data.h"
#include "subsystems/abi.h"
#include "state.h"

/** global AirData state
 */
struct AirData air_data;

/** ABI binding for absolute pressure
 */
#ifndef AIR_DATA_BARO_ABS_ID
#define AIR_DATA_BARO_ABS_ID ABI_BROADCAST
#endif
static abi_event pressure_abs_ev;

/** ABI binding for differential pressure
 */
#ifndef AIR_DATA_BARO_DIFF_ID
#define AIR_DATA_BARO_DIFF_ID ABI_BROADCAST
#endif
static abi_event pressure_diff_ev;

/** Quadratic scale factor for airspeed.
 * airspeed = sqrt(2*p_diff/density)
 * With p_diff in Pa and standard air density of 1.225 kg/m^3,
 * default airspeed scale is 2/1.225
 */
#ifndef AIR_DATA_AIRSPEED_SCALE
#define AIR_DATA_AIRSPEED_SCALE 1.6327
#endif

static void pressure_abs_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure)
{
  air_data.pressure = *pressure;
}

static void pressure_diff_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure)
{
  air_data.differential = *pressure;
  air_data.airspeed = sqrtf(air_data.differential * air_data.airspeed_scale);
#if USE_AIRDATA_AIRSPEED
  stateSetAirspeed_f(&air_data.airspeed);
#endif
}


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_baro_raw(void)
{
  DOWNLINK_SEND_BARO_RAW(DefaultChannel, DefaultDevice,
                         &air_data.pressure, &air_data.differential);
}
#endif

/** AirData initialization. Called at startup.
 *  Bind ABI messages
 */
void air_data_init(void)
{
  air_data.airspeed_scale = AIR_DATA_AIRSPEED_SCALE;

  AbiBindMsgBARO_ABS(AIR_DATA_BARO_ABS_ID, &pressure_abs_ev, pressure_abs_cb);
  AbiBindMsgBARO_ABS(AIR_DATA_BARO_DIFF_ID, &pressure_diff_ev, pressure_diff_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "BARO_RAW", send_baro_raw);
#endif
}

