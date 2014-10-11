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
#include "math/pprz_isa.h"
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

/** Calculate Airspeed from differential pressure by default */
#ifndef AIR_DATA_CALC_AIRSPEED
#define AIR_DATA_CALC_AIRSPEED TRUE
#endif

/** Don't calculate AMSL from baro and QNH by default */
#ifndef AIR_DATA_CALC_AMSL_BARO
#define AIR_DATA_CALC_AMSL_BARO FALSE
#endif


/*
 * Internal variable to keep track of validity.
 */

/** TRUE if QNH has been set */
static bool_t qnh_set;

/** counter to check baro health */
static uint8_t baro_health_counter;


static void pressure_abs_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure)
{
  air_data.pressure = *pressure;

  // calculate QNH from pressure and absolute alitude if that is available
  if (air_data.calc_qnh_once && stateIsGlobalCoordinateValid()) {
    float h = stateGetPositionLla_f()->alt;
    air_data.qnh = pprz_isa_ref_pressure_of_height_full(air_data.pressure, h);
    air_data.calc_qnh_once = FALSE;
    qnh_set = TRUE;
  }

  if (air_data.calc_amsl_baro && qnh_set) {
    air_data.amsl_baro = pprz_isa_height_of_pressure_full(air_data.pressure, air_data.qnh);
    air_data.amsl_baro_valid = TRUE;
  }

  /* reset baro health counter */
  baro_health_counter = 10;
}

static void pressure_diff_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure)
{
  air_data.differential = *pressure;
  if (air_data.calc_airspeed) {
    air_data.airspeed = sqrtf(air_data.differential * air_data.airspeed_scale);
    stateSetAirspeed_f(&air_data.airspeed);
  }
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
  air_data.calc_airspeed = AIR_DATA_CALC_AIRSPEED;
  air_data.calc_amsl_baro = AIR_DATA_CALC_AMSL_BARO;
  air_data.calc_qnh_once = TRUE;
  air_data.amsl_baro_valid = FALSE;

  /* internal variables */
  qnh_set = FALSE;
  baro_health_counter = 0;

  AbiBindMsgBARO_ABS(AIR_DATA_BARO_ABS_ID, &pressure_abs_ev, pressure_abs_cb);
  AbiBindMsgBARO_ABS(AIR_DATA_BARO_DIFF_ID, &pressure_diff_ev, pressure_diff_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "BARO_RAW", send_baro_raw);
#endif
}

float air_data_get_amsl(void)
{
  // If it has be calculated and baro is OK
  if (air_data.amsl_baro_valid) {
    return air_data.amsl_baro;
  }
  // Otherwise use real altitude (from GPS)
  return stateGetPositionLla_f()->alt;
}

void air_data_periodic(void)
{
  // Watchdog on baro
  if (baro_health_counter > 0) {
    baro_health_counter--;
  }
  else {
    air_data.amsl_baro_valid = FALSE;
  }
}
