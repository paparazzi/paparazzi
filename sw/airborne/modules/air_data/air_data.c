/*
 * Copyright (C) 2013 Gautier Hattenberger
 *               2014 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/air_data/air_data.c
 * Air Data interface
 *  - pressures
 *  - airspeed
 *  - angle of attack and sideslip
 *  - wind
 */

#include "modules/air_data/air_data.h"
#include "modules/core/abi.h"
#include "math/pprz_isa.h"
#include "state.h"
#include "generated/airframe.h"

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

/** ABI binding for temperature
 */
#ifndef AIR_DATA_TEMPERATURE_ID
#define AIR_DATA_TEMPERATURE_ID ABI_BROADCAST
#endif
static abi_event temperature_ev;

/** ABI binding for airspeed
 */
#ifndef AIR_DATA_AIRSPEED_ID
#define AIR_DATA_AIRSPEED_ID ABI_BROADCAST
#endif
static abi_event airspeed_ev;

/** ABI binding for incidence angles
 */
#ifndef AIR_DATA_INCIDENCE_ID
#define AIR_DATA_INCIDENCE_ID ABI_BROADCAST
#endif
static abi_event incidence_ev;

/** Default factor to convert estimated airspeed (EAS) to true airspeed (TAS) */
#ifndef AIR_DATA_TAS_FACTOR
#define AIR_DATA_TAS_FACTOR 1.0
#endif

/** Calculate Airspeed from differential pressure by default */
#ifndef AIR_DATA_CALC_AIRSPEED
#define AIR_DATA_CALC_AIRSPEED TRUE
#endif

/** Calculate tas_factor from temp and pressure by default */
#ifndef AIR_DATA_CALC_TAS_FACTOR
#define AIR_DATA_CALC_TAS_FACTOR TRUE
#endif

/** Don't calculate AMSL from baro and QNH by default */
#ifndef AIR_DATA_CALC_AMSL_BARO
#define AIR_DATA_CALC_AMSL_BARO FALSE
#endif


#ifndef USE_AIRSPEED_AIR_DATA
#if USE_AIRSPEED
#define USE_AIRSPEED_AIR_DATA TRUE
PRINT_CONFIG_MSG("USE_AIRSPEED_AIR_DATA automatically set to TRUE")
#endif
#endif

/*
 * Internal variable to keep track of validity.
 */

/** counter to check baro health */
static uint8_t baro_health_counter;


static void pressure_abs_cb(uint8_t __attribute__((unused)) sender_id, uint32_t __attribute__((unused)) stamp, float pressure)
{
  air_data.pressure = pressure;

  // calculate QNH from pressure and absolute altitude if that is available
  if (air_data.calc_qnh_once && stateIsGlobalCoordinateValid()) {
    /// FIXME: use height above MSL (geoid) and not WGS84 ellipsoid here
    // in the meantime use geoid separation at local reference frame origin
    float geoid_separation = 0;
    if (state.ned_initialized_f) {
      geoid_separation = state.ned_origin_f.lla.alt - state.ned_origin_f.hmsl;
    }
    float h = stateGetPositionLla_f()->alt - geoid_separation;
    air_data.qnh = pprz_isa_ref_pressure_of_height_full(air_data.pressure, h) / 100.f;
    air_data.calc_qnh_once = false;
  }

  if (air_data.calc_amsl_baro && air_data.qnh > 0) {
    air_data.amsl_baro = pprz_isa_height_of_pressure_full(air_data.pressure,
                         air_data.qnh * 100.f);
    air_data.amsl_baro_valid = true;
  }

  /* reset baro health counter */
  baro_health_counter = 10;
}

static void pressure_diff_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{
  air_data.differential = pressure;
  if (air_data.calc_airspeed) {
    air_data.airspeed = eas_from_dynamic_pressure(air_data.differential);
    air_data.tas = tas_from_eas(air_data.airspeed);
#if USE_AIRSPEED_AIR_DATA
    stateSetAirspeed_f(air_data.airspeed);
#endif
  }
}

static void temperature_cb(uint8_t __attribute__((unused)) sender_id, float temp)
{
  air_data.temperature = temp;
  /* only calculate tas factor if enabled and we have airspeed and valid data */
  if (air_data.calc_tas_factor && air_data.airspeed > 0 && baro_health_counter > 0 &&
      air_data.pressure > 0) {
    air_data.tas_factor = get_tas_factor(air_data.pressure, air_data.temperature);
  }
}

static void airspeed_cb(uint8_t __attribute__((unused)) sender_id, float eas)
{
  air_data.airspeed = eas;
  if (air_data.calc_airspeed) {
    air_data.tas = tas_from_eas(air_data.airspeed);
#if USE_AIRSPEED_AIR_DATA
    stateSetAirspeed_f(air_data.airspeed);
#endif
  }
}

static void incidence_cb(uint8_t __attribute__((unused)) sender_id, uint8_t flag, float aoa, float sideslip)
{
  if (bit_is_set(flag, 0)) {
    // update angle of attack
    air_data.aoa = aoa;
    stateSetAngleOfAttack_f(aoa);
  }
  if (bit_is_set(flag, 1)) {
    // update sideslip angle
    air_data.sideslip = sideslip;
    stateSetSideslip_f(sideslip);
  }
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_baro_raw(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_BARO_RAW(trans, dev, AC_ID,
                         &air_data.pressure, &air_data.differential);
}

static void send_air_data(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIR_DATA(trans, dev, AC_ID,
                         &air_data.pressure, &air_data.differential,
                         &air_data.temperature, &air_data.qnh,
                         &air_data.amsl_baro, &air_data.airspeed,
                         &air_data.tas);
}

static void send_amsl(struct transport_tx *trans, struct link_device *dev)
{
  const float MeterPerFeet = 0.3048;
  float amsl_baro_ft = air_data.amsl_baro / MeterPerFeet;
  float amsl_gps_ft = stateGetPositionLla_f()->alt / MeterPerFeet;
  pprz_msg_send_AMSL(trans, dev, AC_ID, &amsl_baro_ft, &amsl_gps_ft);
}
#endif

/** AirData initialization. Called at startup.
 *  Bind ABI messages
 */
void air_data_init(void)
{
  air_data.calc_airspeed = AIR_DATA_CALC_AIRSPEED;
  air_data.calc_tas_factor = AIR_DATA_CALC_TAS_FACTOR;
  air_data.calc_amsl_baro = AIR_DATA_CALC_AMSL_BARO;
  air_data.tas_factor = AIR_DATA_TAS_FACTOR;
  air_data.calc_qnh_once = true;
  air_data.amsl_baro_valid = false;

  /* initialize the output variables
   * pressure, qnh, temperature and airspeed to invalid values,
   * rest to zero
   */
  air_data.pressure = -1.0f;
  air_data.qnh = -1.0f;
  air_data.airspeed = -1.0f;
  air_data.tas = -1.0f;
  air_data.temperature = -1000.0f;
  air_data.differential = 0.0f;
  air_data.amsl_baro = 0.0f;
  air_data.aoa = 0.0f;
  air_data.sideslip = 0.0f;
  air_data.wind_speed = 0.0f;
  air_data.wind_dir = 0.0f;

  /* internal variables */
  baro_health_counter = 0;

  AbiBindMsgBARO_ABS(AIR_DATA_BARO_ABS_ID, &pressure_abs_ev, pressure_abs_cb);
  AbiBindMsgBARO_DIFF(AIR_DATA_BARO_DIFF_ID, &pressure_diff_ev, pressure_diff_cb);
  AbiBindMsgTEMPERATURE(AIR_DATA_TEMPERATURE_ID, &temperature_ev, temperature_cb);
  AbiBindMsgAIRSPEED(AIR_DATA_AIRSPEED_ID, &airspeed_ev, airspeed_cb);
  AbiBindMsgINCIDENCE(AIR_DATA_INCIDENCE_ID, &incidence_ev, incidence_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BARO_RAW, send_baro_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIR_DATA, send_air_data);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AMSL, send_amsl);
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
  } else {
    air_data.amsl_baro_valid = false;
  }
}


/**
 * Calculate equivalent airspeed from dynamic pressure.
 * Dynamic pressure @f$q@f$ (also called impact pressure) is the
 * difference between total(pitot) and static pressure.
 *
 * Airspeed from dynamic pressure:
 * @f[ v = \frac12 \sqrt{\frac{2q}{\rho}} @f]
 * with @f$\rho@f$ as air density.
 * Using standard sea level air density @f$\rho_0@f$ gives you equivalent airspeed (EAS).
 *
 * @param q dynamic pressure in Pa
 * @return equivalent airspeed in m/s
 */
float eas_from_dynamic_pressure(float q)
{
  /* q (dynamic pressure) = total pressure - static pressure
   * q = 1/2*rho*speed^2
   * speed = sqrt(2*q/rho)
   * With rho = air density at sea level.
   * Lower bound of q at zero, no flying backwards guys...
   */
  const float two_div_rho_0 = 2.0 / PPRZ_ISA_AIR_DENSITY;
  return sqrtf(Max(q * two_div_rho_0, 0));
}

/**
 * Calculate true airspeed (TAS) factor.
 * TAS = tas_factor * EAS
 *
 * True airspeed (TAS) from equivalent airspeed (EAS):
 * @f[\mbox{TAS} = \mbox{EAS} \sqrt{\frac{\rho_0}{\rho}}@f]
 * and @f$ \frac{\rho_0}{\rho} = \frac{p_0T}{pT_0}@f$ where
 * - @f$p@f$ is the air pressure at the flight condition
 * - @f$p_0@f$ is the air pressure at sea level = 101325 Pa
 * - @f$T@f$ is the air temperature at the flight condition
 * - @f$T_0@f$ is the air temperature at sea level = 288.15 K
 *
 * @param p current air pressure in Pa
 * @param t current air temperature in degrees Celcius
 * @return tas factor
 */
float get_tas_factor(float p, float t)
{
  /* factor to convert EAS to TAS:
   * sqrt(rho0 / rho) = sqrt((p0 * T) / (p * T0))
   * convert input temp to Kelvin
   */
  return sqrtf((PPRZ_ISA_SEA_LEVEL_PRESSURE * KelvinOfCelsius(t)) /
               (p * PPRZ_ISA_SEA_LEVEL_TEMP));
}

/**
 * Calculate true airspeed from equivalent airspeed.
 *
 * True airspeed (TAS) from EAS:
 * TAS = air_data.tas_factor * EAS
 *
 * @param eas equivalent airspeed (EAS) in m/s
 * @return true airspeed in m/s
 */
float tas_from_eas(float eas)
{
  // update tas factor if requested
  if (air_data.calc_tas_factor) {
    if (air_data.pressure > 0.f && air_data.temperature > -900.f) {
      // compute air density from pressure and temperature
      air_data.tas_factor = get_tas_factor(air_data.pressure, air_data.temperature);
    }
    else {
      // compute air density from altitude in ISA condition
      const float z = air_data_get_amsl();
      const float p = pprz_isa_pressure_of_altitude(z);
      const float t = pprz_isa_temperature_of_altitude(z);
      air_data.tas_factor = get_tas_factor(p, CelsiusOfKelvin(t));
    }
  }
  return air_data.tas_factor * eas;
}

/**
 * Calculate true airspeed from dynamic pressure.
 * Dynamic pressure @f$q@f$ (also called impact pressure) is the
 * difference between total(pitot) and static pressure.
 *
 * @param q dynamic pressure in Pa
 * @return true airspeed in m/s
 */
float tas_from_dynamic_pressure(float q)
{
  return tas_from_eas(eas_from_dynamic_pressure(q));
}
