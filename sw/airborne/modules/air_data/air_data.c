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
#include "autopilot.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"
#include "generated/modules.h"
#include "pprzlink/dl_protocol.h"
#include "modules/datalink/downlink.h"
#include <string.h>

/** Calibration of the sensors in the airframe, for example
 * <define name="DIFF_CALIB" value="{{.abi_id=MS45XX_SENDER_ID, .calibrated={.offset=true}, .offset=12.3}}"/>
 * in a section with prefix AIR_DATA_. The order sets the priority of the automatic selection.
 */
#ifndef AIR_DATA_BARO_CALIB
#define AIR_DATA_BARO_CALIB {}
#endif
#ifndef AIR_DATA_DIFF_CALIB
#define AIR_DATA_DIFF_CALIB {}
#endif
#ifndef AIR_DATA_AIRSPEED_CALIB
#define AIR_DATA_AIRSPEED_CALIB {}
#endif
#ifndef AIR_DATA_TEMP_CALIB
#define AIR_DATA_TEMP_CALIB {}
#endif

/** Sensors to use (ABI id), ABI_BROADCAST for the first healthy one */
#ifndef AIR_DATA_BARO_ID
#define AIR_DATA_BARO_ID ABI_BROADCAST
#endif
#ifndef AIR_DATA_AIRSPEED_ID
#define AIR_DATA_AIRSPEED_ID ABI_BROADCAST
#endif
#ifndef AIR_DATA_TEMP_ID
#define AIR_DATA_TEMP_ID ABI_BROADCAST
#endif

/** Also send every calibrated sensor with its own ABI id, besides the one in use with AIR_DATA_SENDER_ID */
#ifndef AIR_DATA_SEND_ALL_SENSORS
#define AIR_DATA_SEND_ALL_SENSORS FALSE
#endif

/** A sensor is unhealthy without a valid value for AIR_DATA_TIMEOUT_FACTOR times its interval,
 * and at least AIR_DATA_TIMEOUT (s), so the slow sensors (1-2 Hz) stay healthy
 */
#ifndef AIR_DATA_TIMEOUT
#define AIR_DATA_TIMEOUT 0.5f
#endif
#ifndef AIR_DATA_TIMEOUT_FACTOR
#define AIR_DATA_TIMEOUT_FACTOR 5.f
#endif

/** Number of values to line up a barometer that starts after the calibration */
#ifndef AIR_DATA_ALIGN_NB
#define AIR_DATA_ALIGN_NB 20
#endif

/** Maximum differential pressure (Pa), about 130 m/s */
#ifndef AIR_DATA_DIFF_MAX
#define AIR_DATA_DIFF_MAX 10000.f
#endif

/** Calibration time (s) */
#ifndef AIR_DATA_CALIB_TIME
#define AIR_DATA_CALIB_TIME 2.f
#endif

/** Calibrate at boot the sensors without an offset in the airframe, after AIR_DATA_CALIB_BOOT_DELAY (s) */
#ifndef AIR_DATA_CALIB_AT_BOOT
#define AIR_DATA_CALIB_AT_BOOT TRUE
#endif
#ifndef AIR_DATA_CALIB_BOOT_DELAY
#define AIR_DATA_CALIB_BOOT_DELAY 3.f
#endif

/** Allow the calibration while the motors are armed */
#ifndef AIR_DATA_CALIB_ARMED
#define AIR_DATA_CALIB_ARMED FALSE
#endif

/** Airspeed estimates (not sensors): only used when no sensor is healthy, and not sent as AIRSPEED */
#define AIR_DATA_IS_ESTIMATE(_id) ((_id) == AIRSPEED_WE_QUAD_ID)

/** Set the airspeed in the state interface */
#ifndef USE_AIRSPEED_AIR_DATA
#define USE_AIRSPEED_AIR_DATA TRUE
#endif

/** global AirData state
 * Statically initialized, the persistent settings are loaded before air_data_init()
 */
struct AirData air_data = {
  .baros = AIR_DATA_BARO_CALIB,
  .diffs = AIR_DATA_DIFF_CALIB,
  .airspeeds = AIR_DATA_AIRSPEED_CALIB,
  .temps = AIR_DATA_TEMP_CALIB,
  .baro_id = AIR_DATA_BARO_ID,
  .airspeed_id = AIR_DATA_AIRSPEED_ID,
  .temp_id = AIR_DATA_TEMP_ID,
  .calib_armed = AIR_DATA_CALIB_ARMED,
};

static abi_event baro_raw_ev, diff_raw_ev, airspeed_raw_ev, temp_raw_ev, incidence_ev;

/** counter to check baro health */
static uint8_t baro_health_counter;

/** Calibration state */
static bool calib_running = false;  ///< Collecting the raw values
static bool calib_boot = false;     ///< Calibration at boot: keep the offsets from the airframe
static float calib_start = 0.f;     ///< Start time of the calibration (s)

/*
 * Sensors
 */

struct air_data_sensor_t *air_data_get_sensor(struct air_data_sensor_t *sensors, uint8_t abi_id, bool create)
{
  if (abi_id == ABI_DISABLE || abi_id == ABI_BROADCAST) {
    return NULL;
  }
  // First the sensor itself (for example from the airframe calibration), then a free slot
  for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
    if (sensors[i].abi_id == abi_id) {
      return &sensors[i];
    }
  }
  if (create) {
    for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
      if (sensors[i].abi_id == ABI_DISABLE) {
        sensors[i].abi_id = abi_id;
        if (sensors[i].scale == 0.f) {
          sensors[i].scale = 1.f;
        }
        return &sensors[i];
      }
    }
  }
  return NULL;
}

/** Store a new raw value of a sensor */
static void air_data_sensor_update(struct air_data_sensor_t *s, uint32_t stamp, float raw, float now)
{
  if (s->time > 0.f) {
    float dt = now - s->time;
    s->dt = (s->dt > 0.f) ? 0.9f * s->dt + 0.1f * dt : dt;
  }
  s->raw = raw;
  s->value = (raw - s->offset) * s->scale;
  s->stamp = stamp;
  s->time = now;
  if (calib_running) {
    s->calib_sum += raw;
    s->calib_cnt++;
  }
}

/** Is the sensor sending valid values */
static bool air_data_sensor_healthy(struct air_data_sensor_t *s, float now)
{
  return s->abi_id != ABI_DISABLE && s->time > 0.f &&
         (now - s->time) < Max(AIR_DATA_TIMEOUT, AIR_DATA_TIMEOUT_FACTOR * s->dt);
}

/** Select a sensor: the one of the setting if healthy, otherwise the first healthy calibrated one,
 * otherwise the first healthy one (a barometer that starts late is lined up before it is used)
 */
static struct air_data_sensor_t *air_data_select(struct air_data_sensor_t *sensors, uint8_t id, float now)
{
  struct air_data_sensor_t *first = NULL, *first_calib = NULL;
  for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
    struct air_data_sensor_t *s = &sensors[i];
    if (!air_data_sensor_healthy(s, now)) {
      continue;
    }
    if (s->abi_id == id) {
      return s;
    }
    if (first == NULL) {
      first = s;
    }
    if (first_calib == NULL && s->calibrated.offset) {
      first_calib = s;
    }
  }
  return (first_calib != NULL) ? first_calib : first;
}

/** Select the airspeed: the sensor of the setting if healthy, otherwise the first healthy differential pressure,
 * then airspeed sensor, then an airspeed estimate
 */
static struct air_data_sensor_t *air_data_select_airspeed(float now)
{
  struct air_data_sensor_t *first = NULL, *estimate = NULL;
  for (uint8_t i = 0; i < 2 * AIR_DATA_MAX_SENSORS; i++) {
    struct air_data_sensor_t *s = (i < AIR_DATA_MAX_SENSORS) ? &air_data.diffs[i] : &air_data.airspeeds[i - AIR_DATA_MAX_SENSORS];
    if (!air_data_sensor_healthy(s, now)) {
      continue;
    }
    if (s->abi_id == air_data.airspeed_id) {
      return s;
    }
    if (AIR_DATA_IS_ESTIMATE(s->abi_id)) {
      if (estimate == NULL) {
        estimate = s;
      }
    } else if (first == NULL) {
      first = s;
    }
  }
  return (first != NULL) ? first : estimate;
}

/** Select the temperature: the one of the setting if healthy, otherwise an air temperature probe,
 * then a differential pressure sensor, then a barometer (never an IMU)
 */
static struct air_data_sensor_t *air_data_select_temp(float now)
{
  static const uint8_t order[] = {TEMPERATURE_TYPE_AIR, TEMPERATURE_TYPE_DIFF, TEMPERATURE_TYPE_BARO};
  struct air_data_sensor_t *best = NULL;
  uint8_t best_rank = sizeof(order);
  for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
    struct air_data_sensor_t *s = &air_data.temps[i];
    if (!air_data_sensor_healthy(s, now)) {
      continue;
    }
    if (s->abi_id == air_data.temp_id) {
      return s;
    }
    for (uint8_t r = 0; r < best_rank; r++) {
      if (s->type == order[r]) {
        best = s;
        best_rank = r;
        break;
      }
    }
  }
  return best;
}

/** Temperature of a sensor (sent with the same ABI id), NAN if unknown */
static float air_data_sensor_temp(uint8_t abi_id)
{
  struct air_data_sensor_t *t = air_data_get_sensor(air_data.temps, abi_id, false);
  return (t != NULL && t->time > 0.f) ? t->value : NAN;
}

/*
 * Outputs of the sensors in use
 */

static void air_data_set_pressure(uint32_t stamp, float pressure)
{
  air_data.pressure = pressure;

  // calculate QNH from pressure and absolute altitude if that is available
  if (air_data.calc_qnh_once && stateIsGlobalCoordinateValid()) {
    /// FIXME: use height above MSL (geoid) and not WGS84 ellipsoid here
    // in the meantime use geoid separation at local reference frame origin
    float geoid_separation = 0;
    if (state.ned_initialized_f) {
      geoid_separation = stateGetLlaOrigin_f().alt - stateGetHmslOrigin_f();
    }
    float h = stateGetPositionLla_f()->alt - geoid_separation;
    air_data.qnh = pprz_isa_ref_pressure_of_height_full(air_data.pressure, h) / 100.f;
    air_data.calc_qnh_once = false;
  }

  if (air_data.qnh > 0) {
    air_data.amsl_baro = pprz_isa_height_of_pressure_full(air_data.pressure,
                         air_data.qnh * 100.f);
    air_data.amsl_baro_valid = true;
  }

  /* reset baro health counter */
  baro_health_counter = 10;

  AbiSendMsgBARO_ABS(AIR_DATA_SENDER_ID, stamp, pressure);
}

static void air_data_set_airspeed(float eas, bool send)
{
  air_data.airspeed = eas;
  air_data.tas = tas_from_eas(air_data.airspeed);
#if USE_AIRSPEED_AIR_DATA
  stateSetAirspeed_f(MODULE_AIR_DATA_ID, air_data.airspeed);
#endif
  if (send) {
    AbiSendMsgAIRSPEED(AIR_DATA_SENDER_ID, air_data.airspeed);
  }
}

/*
 * ABI callbacks of the drivers
 */

static void baro_raw_cb(uint8_t sender_id, uint32_t stamp, float pressure)
{
  // bad pressure (Pa), don't use
  if (!isfinite(pressure) || pressure < 1000.f || pressure > 120000.f) {
    return;
  }
  struct air_data_sensor_t *s = air_data_get_sensor(air_data.baros, sender_id, true);
  if (s == NULL) {
    return;
  }
  float now = get_sys_time_float();
  air_data_sensor_update(s, stamp, pressure, now);
#if AIR_DATA_SEND_ALL_SENSORS
  AbiSendMsgBARO_ABS(sender_id, stamp, s->value);
#endif

  struct air_data_sensor_t *sel = air_data_select(air_data.baros, air_data.baro_id, now);
  air_data.baro_used = (sel != NULL) ? sel->abi_id : ABI_DISABLE;
  if (sel == s) {
    air_data_set_pressure(stamp, s->value);
  } else if (!calib_running && !s->calibrated.offset && sel != NULL && sel->calibrated.offset) {
    // a barometer that starts after the calibration: line it up with the one in use
    s->calib_sum += pressure - air_data.pressure / s->scale;
    s->calib_cnt++;
    if (s->calib_cnt >= AIR_DATA_ALIGN_NB) {
      s->offset = s->calib_sum / s->calib_cnt;
      s->calibrated.offset = true;
    }
  }
}

static void diff_raw_cb(uint8_t sender_id, uint32_t stamp, float pressure)
{
  if (!isfinite(pressure) || fabsf(pressure) > AIR_DATA_DIFF_MAX) {
    return;
  }
  struct air_data_sensor_t *s = air_data_get_sensor(air_data.diffs, sender_id, true);
  if (s == NULL) {
    return;
  }
  float now = get_sys_time_float();
  air_data_sensor_update(s, stamp, pressure, now);
#if AIR_DATA_SEND_ALL_SENSORS
  AbiSendMsgBARO_DIFF(sender_id, s->value);
  AbiSendMsgAIRSPEED(sender_id, eas_from_dynamic_pressure(s->value));
#endif

  struct air_data_sensor_t *sel = air_data_select_airspeed(now);
  air_data.airspeed_used = (sel != NULL) ? sel->abi_id : ABI_DISABLE;
  if (sel == s) {
    air_data.differential = s->value;
    AbiSendMsgBARO_DIFF(AIR_DATA_SENDER_ID, s->value);
    air_data_set_airspeed(eas_from_dynamic_pressure(s->value), true);
  }
}

static void airspeed_raw_cb(uint8_t sender_id, uint32_t stamp, float eas)
{
  if (!isfinite(eas)) {
    return;
  }
  struct air_data_sensor_t *s = air_data_get_sensor(air_data.airspeeds, sender_id, true);
  if (s == NULL) {
    return;
  }
  float now = get_sys_time_float();
  air_data_sensor_update(s, stamp, eas, now);
#if AIR_DATA_SEND_ALL_SENSORS
  if (!AIR_DATA_IS_ESTIMATE(sender_id)) {
    AbiSendMsgAIRSPEED(sender_id, s->value);
  }
#endif

  struct air_data_sensor_t *sel = air_data_select_airspeed(now);
  air_data.airspeed_used = (sel != NULL) ? sel->abi_id : ABI_DISABLE;
  if (sel == s) {
    // an estimate isn't sent, the estimators (e.g. ins_ekf2) would fuse it
    air_data_set_airspeed(s->value, !AIR_DATA_IS_ESTIMATE(sender_id));
  }
}

static void temp_raw_cb(uint8_t sender_id, uint32_t stamp, float temp, uint8_t type)
{
  if (!isfinite(temp) || temp < -100.f || temp > 150.f) {
    return;
  }
  // the IMU and other temperatures don't take a slot, unless selected
  if ((type == TEMPERATURE_TYPE_IMU || type == TEMPERATURE_TYPE_OTHER) && sender_id != air_data.temp_id) {
    return;
  }
  struct air_data_sensor_t *s = air_data_get_sensor(air_data.temps, sender_id, true);
  if (s == NULL) {
    return;
  }
  s->type = type;
  float now = get_sys_time_float();
  air_data_sensor_update(s, stamp, temp, now);

  struct air_data_sensor_t *sel = air_data_select_temp(now);
  air_data.temp_used = (sel != NULL) ? sel->abi_id : ABI_DISABLE;
  if (sel == s) {
    air_data.temperature = s->value;
    AbiSendMsgTEMPERATURE(AIR_DATA_SENDER_ID, air_data.temperature);
  }
}

static void incidence_cb(uint8_t __attribute__((unused)) sender_id, uint8_t flag, float aoa, float sideslip)
{
  if (bit_is_set(flag, 0)) {
    // update angle of attack
    air_data.aoa = aoa;
    stateSetAngleOfAttack_f(MODULE_AIR_DATA_ID, aoa);
  }
  if (bit_is_set(flag, 1)) {
    // update sideslip angle
    air_data.sideslip = sideslip;
    stateSetSideslip_f(MODULE_AIR_DATA_ID, sideslip);
  }
}

/*
 * Calibration
 */

static void air_data_start_calibration(bool boot)
{
  air_data.calibrate = false;
  if (autopilot_get_motors_on() && !air_data.calib_armed) {
    // refused while armed
#if DOWNLINK
    char msg[] = "Air data calibration refused: armed";
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(msg), msg);
#endif
    return;
  }
  struct air_data_sensor_t *sensors[] = {air_data.baros, air_data.diffs};
  for (uint8_t k = 0; k < 2; k++) {
    for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
      sensors[k][i].calib_sum = 0.f;
      sensors[k][i].calib_cnt = 0;
    }
  }
  calib_boot = boot;
  calib_start = get_sys_time_float();
  calib_running = true;
  air_data.calibrate = true;
}

/** Zero the differential pressures and line up the barometers with the one in use */
static void air_data_finish_calibration(void)
{
  calib_running = false;
  air_data.calibrate = false;

  for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
    struct air_data_sensor_t *s = &air_data.diffs[i];
    if (s->calib_cnt > 0 && !(calib_boot && s->calibrated.offset)) {
      s->offset = s->calib_sum / s->calib_cnt;
      s->calibrated.offset = true;
    }
  }

  struct air_data_sensor_t *ref = air_data_select(air_data.baros, air_data.baro_id, get_sys_time_float());
  if (ref != NULL && ref->calib_cnt > 0) {
    float p_ref = (ref->calib_sum / ref->calib_cnt - ref->offset) * ref->scale;
    air_data.qfe = p_ref;
    ref->calibrated.offset = true;
    for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
      struct air_data_sensor_t *s = &air_data.baros[i];
      if (s != ref && s->calib_cnt > 0 && !(calib_boot && s->calibrated.offset)) {
        s->offset = s->calib_sum / s->calib_cnt - p_ref / s->scale;
        s->calibrated.offset = true;
      }
    }
    air_data.calc_qnh_once = true;
  }
}

void air_data_Calibrate(bool calibrate)
{
  if (calibrate && !calib_running) {
    air_data_start_calibration(false);
  }
}

/*
 * Checks and telemetry
 */

#if PREFLIGHT_CHECKS
#include "modules/checks/preflight_checks.h"

static struct preflight_check_t air_data_pfc;

static void air_data_preflight(struct preflight_result_t *result)
{
  uint16_t fail_cnt = result->fail_cnt;
  if (calib_running) {
    preflight_error(result, "Air data calibrating");
  }
  for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
    if (air_data.diffs[i].abi_id != ABI_DISABLE && !air_data.diffs[i].calibrated.offset) {
      preflight_error(result, "Airspeed sensor %d not zeroed", air_data.diffs[i].abi_id);
    }
  }
  if (result->fail_cnt == fail_cnt) {
    preflight_success(result, "Air data calibrated");
  }
}
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/** Next sensor in use for the round-robin telemetry, NULL if none */
static struct air_data_sensor_t *air_data_next(struct air_data_sensor_t *sensors, uint8_t *idx)
{
  for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
    uint8_t j = (*idx + i) % AIR_DATA_MAX_SENSORS;
    if (sensors[j].abi_id != ABI_DISABLE) {
      *idx = (j + 1) % AIR_DATA_MAX_SENSORS;
      return &sensors[j];
    }
  }
  return NULL;
}

static void send_baro_raw(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t idx = 0;
  struct air_data_sensor_t *s = air_data_next(air_data.baros, &idx);
  if (s == NULL) {
    return;
  }
  float temp = air_data_sensor_temp(s->abi_id);
  pprz_msg_send_BARO_RAW(trans, dev, AC_ID, &s->abi_id, &s->raw, &s->value, &s->offset, &temp);
}

static void send_airspeed_raw(struct transport_tx *trans, struct link_device *dev)
{
  // the differential pressure sensors, then the airspeed sensors
  static uint8_t idx = 0;
  struct air_data_sensor_t *s = NULL;
  bool is_diff = false;
  for (uint8_t i = 0; i < 2 * AIR_DATA_MAX_SENSORS && s == NULL; i++) {
    uint8_t j = (idx + i) % (2 * AIR_DATA_MAX_SENSORS);
    is_diff = (j < AIR_DATA_MAX_SENSORS);
    struct air_data_sensor_t *c = is_diff ? &air_data.diffs[j] : &air_data.airspeeds[j - AIR_DATA_MAX_SENSORS];
    if (c->abi_id != ABI_DISABLE) {
      s = c;
      idx = (j + 1) % (2 * AIR_DATA_MAX_SENSORS);
    }
  }
  if (s == NULL) {
    return;
  }
  uint16_t raw = 0;
  float temp = air_data_sensor_temp(s->abi_id);
  float diff = is_diff ? s->value : 0.f;
  float airspeed = is_diff ? eas_from_dynamic_pressure(s->value) : s->value;
  pprz_msg_send_AIRSPEED_RAW(trans, dev, AC_ID, &s->abi_id, &raw, &s->offset, &diff, &temp, &airspeed);
}

static void send_air_data(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIR_DATA(trans, dev, AC_ID,
                         &air_data.pressure, &air_data.differential,
                         &air_data.temperature, &air_data.qnh,
                         &air_data.amsl_baro, &air_data.airspeed,
                         &air_data.tas, &air_data.baro_used, &air_data.airspeed_used);
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
  /* the sensors of the airframe calibration, the settings are statically initialized */
  for (uint8_t i = 0; i < AIR_DATA_MAX_SENSORS; i++) {
    struct air_data_sensor_t *sensors[] = {&air_data.baros[i], &air_data.diffs[i], &air_data.airspeeds[i], &air_data.temps[i]};
    for (uint8_t k = 0; k < 4; k++) {
      if (sensors[k]->scale == 0.f) {
        sensors[k]->scale = 1.f;
      }
    }
  }
  air_data.baro_used = ABI_DISABLE;
  air_data.airspeed_used = ABI_DISABLE;
  air_data.temp_used = ABI_DISABLE;
  air_data.calibrate = false;
  air_data.qfe = -1.f;
  air_data.tas_factor = 1.f;
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

  AbiBindMsgBARO_ABS_RAW(ABI_BROADCAST, &baro_raw_ev, baro_raw_cb);
  AbiBindMsgBARO_DIFF_RAW(ABI_BROADCAST, &diff_raw_ev, diff_raw_cb);
  AbiBindMsgAIRSPEED_RAW(ABI_BROADCAST, &airspeed_raw_ev, airspeed_raw_cb);
  AbiBindMsgTEMPERATURE_RAW(ABI_BROADCAST, &temp_raw_ev, temp_raw_cb);
  AbiBindMsgINCIDENCE(ABI_BROADCAST, &incidence_ev, incidence_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BARO_RAW, send_baro_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_RAW, send_airspeed_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIR_DATA, send_air_data);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AMSL, send_amsl);
#endif

#if PREFLIGHT_CHECKS
  preflight_check_register(&air_data_pfc, air_data_preflight);
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

  // Sensors in use, also when they stop sending
  float now = get_sys_time_float();
  struct air_data_sensor_t *s = air_data_select(air_data.baros, air_data.baro_id, now);
  air_data.baro_used = (s != NULL) ? s->abi_id : ABI_DISABLE;
  s = air_data_select_airspeed(now);
  air_data.airspeed_used = (s != NULL) ? s->abi_id : ABI_DISABLE;
  s = air_data_select_temp(now);
  air_data.temp_used = (s != NULL) ? s->abi_id : ABI_DISABLE;

  // Calibration
#if AIR_DATA_CALIB_AT_BOOT
  static bool boot_calib_done = false;
  if (!boot_calib_done && now > AIR_DATA_CALIB_BOOT_DELAY) {
    boot_calib_done = true;
    if (!calib_running) {
      air_data_start_calibration(true);
    }
  }
#endif
  if (calib_running && (now - calib_start) > AIR_DATA_CALIB_TIME) {
    air_data_finish_calibration();
  }
}

void air_data_parse_WIND_INFO(struct link_device *dev __attribute__((unused)), struct transport_tx *trans __attribute__((unused)), uint8_t *buf)
{
  if (DL_WIND_INFO_ac_id(buf) != AC_ID) { return; }
  uint8_t flags = DL_WIND_INFO_flags(buf);
  struct FloatVect2 wind = { 0.f, 0.f };
  float upwind = 0.f;
  if (bit_is_set(flags, 0)) {
    wind.x = DL_WIND_INFO_north(buf);
    wind.y = DL_WIND_INFO_east(buf);
    stateSetHorizontalWindspeed_f(MODULE_AIR_DATA_ID, &wind);
    air_data.wind_speed = float_vect2_norm(&wind);
    air_data.wind_dir = atan2f(wind.y, wind.x);
  }
  if (bit_is_set(flags, 1)) {
    upwind = DL_WIND_INFO_up(buf);
    stateSetVerticalWindspeed_f(MODULE_AIR_DATA_ID, upwind);
  }
#if !USE_AIRSPEED
  if (bit_is_set(flags, 2)) {
    air_data.tas = DL_WIND_INFO_airspeed(buf);
    air_data.airspeed = eas_from_tas(air_data.tas);
    stateSetAirspeed_f(MODULE_AIR_DATA_ID, air_data.airspeed);
  }
#endif
#ifdef WIND_INFO_RET
  float airspeed = stateGetAirspeed_f();
  pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID, &flags, &wind.y, &wind.x, &upwind, &airspeed);
#endif
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
  float sign = 1.0f;
  if (q < 0) {
    sign = -1.0f;
    q = -q;
  }
  return sqrtf(q * two_div_rho_0) * sign;
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
 * Internal utility function to compute the current tas factor
 */
static void compute_tas_factor(void)
{
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
  compute_tas_factor();
  return air_data.tas_factor * eas;
}

/**
 * Calculate equivalent airspeed from true airspeed.
 *
 * EAS from True airspeed (TAS):
 * EAS = TAS / air_data.tas_factor
 *
 * @param tas true airspeed (TAS) in m/s
 * @return equivalent airspeed in m/s
 */
float eas_from_tas(float tas)
{
  compute_tas_factor();
  return tas / air_data.tas_factor;
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
