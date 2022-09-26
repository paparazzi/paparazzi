/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ins/ins_mekf_wind_wrapper.c
 *
 * Paparazzi specific wrapper to run MEKF-Wind INS filter.
 */

#include "modules/ins/ins_mekf_wind_wrapper.h"
#include "modules/ins/ins_mekf_wind.h"
#include "modules/air_data/air_data.h"
#include "modules/ahrs/ahrs_float_utils.h"
#if USE_AHRS_ALIGNER
#include "modules/ahrs/ahrs_aligner.h"
#endif
#include "modules/ins/ins.h"
#include "modules/core/abi.h"
#include "math/pprz_isa.h"
#include "state.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#define MEKF_WIND_USE_UTM TRUE
#if MEKF_WIND_USE_UTM
#include "firmwares/fixedwing/nav.h"
#endif

#ifndef INS_MEKF_WIND_FILTER_ID
#define INS_MEKF_WIND_FILTER_ID 3
#endif

struct InsMekfWind ins_mekf_wind;

/** last accel measurement */
static struct FloatVect3 ins_mekf_wind_accel;
static uint32_t last_imu_stamp = 0;

/** update state interface */
static void set_state_from_ins(void);

/** logging functions */
#if LOG_MEKF_WIND
#ifndef SITL
#include "modules/loggers/sdlog_chibios.h"
#define PrintLog sdLogWriteLog
#define LogFileIsOpen() (pprzLogFile != -1)
#else // SITL: print in a file
#include <stdio.h>
#define PrintLog fprintf
#define LogFileIsOpen() (pprzLogFile != NULL)
static FILE* pprzLogFile = NULL;
#endif
#endif

/** telemetry functions */
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers ltp_to_imu_euler;
  struct FloatQuat quat = ins_mekf_wind_get_quat();
  float_eulers_of_quat(&ltp_to_imu_euler, &quat);
  uint8_t id = INS_MEKF_WIND_FILTER_ID;
  pprz_msg_send_AHRS_EULER(trans, dev, AC_ID,
                           &ltp_to_imu_euler.phi,
                           &ltp_to_imu_euler.theta,
                           &ltp_to_imu_euler.psi,
                           &id);
}

static void send_wind(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_f wind_ned = ins_mekf_wind_get_wind_ned();
  struct EnuCoor_f wind_enu;
  ENU_OF_TO_NED(wind_enu, wind_ned);
  float va = ins_mekf_wind_get_airspeed_norm();
  uint8_t flags = 7; // 3D wind + airspeed
  pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID, &flags,
      &wind_enu.x, &wind_enu.y, &wind_enu.z, &va);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3; // OK
  uint16_t val = 0;
  if (!ins_mekf_wind.is_aligned) {
    mde = 2; // ALIGN
  } else if (!ins_mekf_wind.baro_initialized || !ins_mekf_wind.gps_initialized) {
    mde = 1; // INIT
  }
  uint32_t t_diff = get_sys_time_usec() - last_imu_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) {
    mde = 5; // IMU_LOST
  }
  uint8_t id = INS_MEKF_WIND_FILTER_ID;
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &id, &mde, &val);
}

static void send_inv_filter(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers eulers;
  struct FloatQuat quat = ins_mekf_wind_get_quat();
  float_eulers_of_quat(&eulers, &quat);
  //struct FloatRates rates = ins_mekf_wind_get_body_rates();
  struct NedCoor_f pos = ins_mekf_wind_get_pos_ned();
  struct NedCoor_f speed = ins_mekf_wind_get_speed_ned();
  //struct NedCoor_f accel = ins_mekf_wind_get_accel_ned();
  struct FloatVect3 ab = ins_mekf_wind_get_accel_bias();
  struct FloatRates rb = ins_mekf_wind_get_rates_bias();
  float airspeed = ins_mekf_wind_get_airspeed_norm();
  pprz_msg_send_INV_FILTER(trans, dev,
      AC_ID,
      &quat.qi,
      &eulers.phi,
      &eulers.theta,
      &eulers.psi,
      &speed.x,
      &speed.y,
      &speed.z,
      &pos.x,
      &pos.y,
      &pos.z,
      &rb.p,
      &rb.q,
      &rb.r,
      &ab.x,
      &ab.y,
      &ab.z,
      &airspeed);
}
#endif

/*
 * ABI bindings
 */

/** airspeed (Pitot tube) */
#ifndef INS_MEKF_WIND_AIRSPEED_ID
#define INS_MEKF_WIND_AIRSPEED_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_AIRSPEED_ID)

/** incidence angles */
#ifndef INS_MEKF_WIND_INCIDENCE_ID
#define INS_MEKF_WIND_INCIDENCE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_INCIDENCE_ID)

/** baro */
#ifndef INS_MEKF_WIND_BARO_ID
#if USE_BARO_BOARD
#define INS_MEKF_WIND_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_MEKF_WIND_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_BARO_ID)

/** IMU (gyro, accel) */
#ifndef INS_MEKF_WIND_IMU_ID
#define INS_MEKF_WIND_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_IMU_ID)

/** magnetometer */
#ifndef INS_MEKF_WIND_MAG_ID
#define INS_MEKF_WIND_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_MAG_ID)

/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_MEKF_WIND_GPS_ID
#define INS_MEKF_WIND_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_MEKF_WIND_GPS_ID)

static abi_event pressure_diff_ev;
static abi_event airspeed_ev;
static abi_event incidence_ev;
static abi_event baro_ev;
static abi_event mag_ev;
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event aligner_ev;
static abi_event geo_mag_ev;
static abi_event gps_ev;


static void baro_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float pressure)
{
  static float ins_qfe = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  static float alpha = 10.0f;
  static int32_t i = 1;
  static float baro_moy = 0.0f;
  static float baro_prev = 0.0f;

  if (!ins_mekf_wind.baro_initialized) {
    // try to find a stable qfe
    // TODO generic function in pprz_isa ?
    if (i == 1) {
      baro_moy = pressure;
      baro_prev = pressure;
    }
    baro_moy = (baro_moy * (i - 1) + pressure) / i;
    alpha = (10.*alpha + (baro_moy - baro_prev)) / (11.0f);
    baro_prev = baro_moy;
    // test stop condition
    if (fabs(alpha) < 0.1f) {
      ins_qfe = baro_moy;
      ins_mekf_wind.baro_initialized = true;
    }
    if (i == 250) {
      ins_qfe = pressure;
      ins_mekf_wind.baro_initialized = true;
    }
    i++;
  } else { /* normal update with baro measurement */
    if (ins_mekf_wind.is_aligned && ins_mekf_wind.gps_initialized) {
      float baro_alt = -pprz_isa_height_of_pressure(pressure, ins_qfe); // Z down
      ins_mekf_wind_update_baro(baro_alt);

#if LOG_MEKF_WIND
      if (LogFileIsOpen()) {
        PrintLog(pprzLogFile, "%.3f baro %.3f \n", get_sys_time_float(), baro_alt);
      }
#endif
    }
  }
}

static void pressure_diff_cb(uint8_t __attribute__((unused)) sender_id, float pdyn)
{
  if (ins_mekf_wind.is_aligned) {
    float airspeed = tas_from_dynamic_pressure(pdyn);
    ins_mekf_wind_update_airspeed(airspeed);

#if LOG_MEKF_WIND
    if (LogFileIsOpen()) {
      PrintLog(pprzLogFile, "%.3f airspeed %.3f\n", get_sys_time_float(), airspeed);
    }
#endif
  }
}

static void airspeed_cb(uint8_t __attribute__((unused)) sender_id, float airspeed)
{
  if (ins_mekf_wind.is_aligned) {
    ins_mekf_wind_update_airspeed(tas_from_eas(airspeed));

#if LOG_MEKF_WIND
    if (LogFileIsOpen()) {
      PrintLog(pprzLogFile, "%.3f airspeed %.3f\n", get_sys_time_float(), airspeed);
    }
#endif
  }
}

static void incidence_cb(uint8_t __attribute__((unused)) sender_id, uint8_t flag, float aoa, float sideslip)
{
  if (ins_mekf_wind.is_aligned && bit_is_set(flag, 0) && bit_is_set(flag, 1)) {
    ins_mekf_wind_update_incidence(aoa, sideslip);

#if LOG_MEKF_WIND
    if (LogFileIsOpen()) {
      PrintLog(pprzLogFile, "%.3f incidence %.3f %.3f\n", get_sys_time_float(), aoa, sideslip);
    }
#endif
  }
}

/**
 * Call ins_mekf_wind_propagate on new gyro measurements.
 * Since acceleration measurement is also needed for propagation,
 * use the last stored accel from #ins_mekfw_accel.
 */
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp, struct Int32Rates *gyro)
{
  if (ins_mekf_wind.is_aligned) {
    struct FloatRates gyro_body;
    RATES_FLOAT_OF_BFP(gyro_body, *gyro);

#if USE_AUTO_INS_FREQ || !defined(INS_PROPAGATE_FREQUENCY)
    PRINT_CONFIG_MSG("Calculating dt for INS MEKF_WIND propagation.")

    if (last_imu_stamp > 0) {
      float dt = (float)(stamp - last_imu_stamp) * 1e-6;
      if (ins_mekf_wind.gps_initialized) {
        ins_mekf_wind_propagate(&gyro_body, &ins_mekf_wind_accel, dt);
      } else {
        ins_mekf_wind_propagate_ahrs(&gyro_body, &ins_mekf_wind_accel, dt);
      }
    }
#else
    PRINT_CONFIG_MSG("Using fixed INS_PROPAGATE_FREQUENCY for INS MEKF_WIND propagation.")
      PRINT_CONFIG_VAR(INS_PROPAGATE_FREQUENCY)
      const float dt = 1. / (INS_PROPAGATE_FREQUENCY);
    if (ins_mekf_wind.gps_initialized) {
      ins_mekf_wind_propagate(&gyro_body, &ins_mekf_wind_accel, dt);
    } else {
      ins_mekf_wind_propagate_ahrs(&gyro_body, &ins_mekf_wind_accel, dt);
    }
#endif

    // update state interface
    set_state_from_ins();

#if LOG_MEKF_WIND
    if (LogFileIsOpen()) {
      float time = get_sys_time_float();

      PrintLog(pprzLogFile,
          "%.3f gyro_accel %.3f %.3f %.3f %.3f %.3f %.3f \n",
          time,
          gyro_body.p, gyro_body.q, gyro_body.r,
          ins_mekf_wind_accel.x,
          ins_mekf_wind_accel.y,
          ins_mekf_wind_accel.z);

      struct FloatEulers eulers;
      struct FloatQuat quat = ins_mekf_wind_get_quat();
      float_eulers_of_quat(&eulers, &quat);
      struct FloatRates rates = ins_mekf_wind_get_body_rates();
      struct NedCoor_f pos = ins_mekf_wind_get_pos_ned();
      struct NedCoor_f speed = ins_mekf_wind_get_speed_ned();
      struct NedCoor_f accel = ins_mekf_wind_get_accel_ned();
      struct FloatVect3 ab = ins_mekf_wind_get_accel_bias();
      struct FloatRates rb = ins_mekf_wind_get_rates_bias();
      float bb = ins_mekf_wind_get_baro_bias();
      struct NedCoor_f wind = ins_mekf_wind_get_wind_ned();
      float airspeed = ins_mekf_wind_get_airspeed_norm();
      PrintLog(pprzLogFile,
          "%.3f output %.4f %.4f %.4f %.3f %.3f %.3f ",
          time,
          eulers.phi, eulers.theta, eulers.psi,
          rates.p, rates.q, rates.r);
      PrintLog(pprzLogFile,
          "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f ",
          pos.x, pos.y, pos.z,
          speed.x, speed.y, speed.z,
          accel.x, accel.y, accel.z);
      PrintLog(pprzLogFile,
          "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
          ab.x, ab.y, ab.z, rb.p, rb.q, rb.r, bb,
          wind.x, wind.y, wind.z, airspeed);
    }
#endif
  }

  /* timestamp in usec when last callback was received */
  last_imu_stamp = stamp;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  ACCELS_FLOAT_OF_BFP(ins_mekf_wind_accel, *accel);
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ins_mekf_wind.is_aligned) {
    struct FloatVect3 mag_body;
    MAGS_FLOAT_OF_BFP(mag_body, *mag);

    // only correct attitude if GPS is not initialized
    ins_mekf_wind_update_mag(&mag_body, !ins_mekf_wind.gps_initialized);

#if LOG_MEKF_WIND
    if (LogFileIsOpen()) {
      PrintLog(pprzLogFile,
          "%.3f magneto %.3f %.3f %.3f\n",
          get_sys_time_float(),
          mag_body.x, mag_body.y, mag_body.z);
    }
#endif
  }
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ins_mekf_wind.is_aligned) {
    struct FloatRates gyro_body;
    RATES_FLOAT_OF_BFP(gyro_body, *lp_gyro);

    struct FloatVect3 accel_body;
    ACCELS_FLOAT_OF_BFP(accel_body, *lp_accel);

    struct FloatVect3 mag_body;
    MAGS_FLOAT_OF_BFP(mag_body, *lp_mag);

    struct FloatQuat quat;
    ahrs_float_get_quat_from_accel_mag(&quat, &accel_body, &mag_body);
    ins_mekf_wind_align(&gyro_body, &quat);
    // udate state interface
    set_state_from_ins();

    // ins and ahrs are now running
    ins_mekf_wind.is_aligned = true;
  }
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  ins_mekf_wind_set_mag_h(h);
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
	if (ins_mekf_wind.is_aligned && gps_s->fix >= GPS_FIX_3D) {

#if MEKF_WIND_USE_UTM
		if (state.utm_initialized_f) {
			struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);
      struct FloatVect3 pos, speed;
			// position (local ned)
			pos.x = utm.north - state.utm_origin_f.north;
			pos.y = utm.east - state.utm_origin_f.east;
			pos.z = state.utm_origin_f.alt - utm.alt;
			// speed
			speed.x = gps_s->ned_vel.x / 100.0f;
			speed.y = gps_s->ned_vel.y / 100.0f;
			speed.z = gps_s->ned_vel.z / 100.0f;
      if (!ins_mekf_wind.gps_initialized) {
        ins_mekf_wind_set_pos_ned((struct NedCoor_f*)(&pos));
        ins_mekf_wind_set_speed_ned((struct NedCoor_f*)(&speed));
        ins_mekf_wind.gps_initialized = true;
      }
      ins_mekf_wind_update_pos_speed(&pos, &speed);

#if LOG_MEKFW_FILTER
      if (LogFileIsOpen()) {
        PrintLog(pprzLogFile,
            "%.3f gps %.3f %.3f %.3f %.3f %.3f %.3f \n",
            get_sys_time_float(),
            pos.x, pos.y, pos.z, speed.x, speed.y, speed.z);
      }
#endif
		}

#else
		if (state.ned_initialized_f) {
      struct FloatVect3 pos, speed;
			struct NedCoor_i gps_pos_cm_ned, ned_pos;
			ned_of_ecef_point_i(&gps_pos_cm_ned, &state.ned_origin_i, &gps_s->ecef_pos);
			INT32_VECT3_SCALE_2(ned_pos, gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
			NED_FLOAT_OF_BFP(pos, ned_pos);
			struct EcefCoor_f ecef_vel;
			ECEF_FLOAT_OF_BFP(ecef_vel, gps_s->ecef_vel);
			ned_of_ecef_vect_f(&speed, &state.ned_origin_f, &ecef_vel);
      ins_mekf_wind_update_pos_speed(&pos, &speed);

#if LOG_MEKFW_FILTER
      if (LogFileIsOpen()) {
        PrintLog(pprzLogFile,
            "%.3f gps %.3f %.3f %.3f %.3f %.3f %.3f \n",
            get_sys_time_float(),
            pos.x, pos.y, pos.z, speed.x, speed.y, speed.z);
      }
#endif
		}
#endif
	}
}

/**
 * Set current state (ltp to body orientation/rates and ned pos/speed/accel
 */
static void set_state_from_ins(void)
{
  struct FloatQuat quat = ins_mekf_wind_get_quat();
  stateSetNedToBodyQuat_f(&quat);

  struct FloatRates rates = ins_mekf_wind_get_body_rates();
  stateSetBodyRates_f(&rates);

  struct NedCoor_f pos = ins_mekf_wind_get_pos_ned();
  stateSetPositionNed_f(&pos);

  struct NedCoor_f speed = ins_mekf_wind_get_speed_ned();
  stateSetSpeedNed_f(&speed);

  struct NedCoor_f accel = ins_mekf_wind_get_accel_ned();
  stateSetAccelNed_f(&accel);
}

/**
 * Init function
 */
void ins_mekf_wind_wrapper_init(void)
{
  // init position
#if MEKF_WIND_USE_UTM
  struct UtmCoor_f utm0;
  utm0.north = (float)nav_utm_north0;
  utm0.east = (float)nav_utm_east0;
  utm0.alt = GROUND_ALT;
  utm0.zone = nav_utm_zone0;
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);
#else
  struct LlaCoor_i llh_nav0;
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);
  ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ltp_def);
#endif

  // reset flags
  ins_mekf_wind.is_aligned = false;
  ins_mekf_wind.reset = false;
  ins_mekf_wind.baro_initialized = false;
  ins_mekf_wind.gps_initialized = false;

  // aligner
#if USE_AHRS_ALIGNER
  ahrs_aligner_init();
#endif

  // init filter
  ins_mekf_wind_init();
  const struct FloatVect3 mag_h = { INS_H_X, INS_H_Y, INS_H_Z };
  ins_mekf_wind_set_mag_h(&mag_h);

  // Bind to ABI messages
  AbiBindMsgBARO_ABS(INS_MEKF_WIND_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgBARO_DIFF(INS_MEKF_WIND_AIRSPEED_ID, &pressure_diff_ev, pressure_diff_cb);
  AbiBindMsgAIRSPEED(INS_MEKF_WIND_AIRSPEED_ID, &airspeed_ev, airspeed_cb);
  AbiBindMsgINCIDENCE(INS_MEKF_WIND_INCIDENCE_ID, &incidence_ev, incidence_cb);
  AbiBindMsgIMU_MAG(INS_MEKF_WIND_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_GYRO(INS_MEKF_WIND_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(INS_MEKF_WIND_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_LOWPASSED(INS_MEKF_WIND_IMU_ID, &aligner_ev, aligner_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);
  AbiBindMsgGPS(INS_MEKF_WIND_GPS_ID, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER, send_euler);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_INFO_RET, send_wind);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INV_FILTER, send_inv_filter);
#endif

  // init log
#if LOG_MEKF_WIND && SITL
  // open log file for writing
  // path should be specified in airframe file
  uint32_t counter = 0;
  char filename[512];
  snprintf(filename, 512, "%s/mekf_wind_%05d.csv", STRINGIFY(MEKF_WIND_LOG_PATH), counter);
  // check availale name
  while ((pprzLogFile = fopen(filename, "r"))) {
    fclose(pprzLogFile);
    snprintf(filename, 512, "%s/mekf_wind_%05d.csv", STRINGIFY(MEKF_WIND_LOG_PATH), ++counter);
  }
  pprzLogFile = fopen(filename, "w");
  if (pprzLogFile == NULL) {
    printf("Failed to open WE log file '%s'\n",filename);
  } else {
    printf("Opening WE log file '%s'\n",filename);
  }
#endif

}

/**
 * local implemetation of the ins_reset functions
 */

void ins_reset_local_origin(void)
{
#if MEKF_WIND_USE_UTM
  struct UtmCoor_f utm = utm_float_from_gps(&gps, 0);
  // reset state UTM ref
  stateSetLocalUtmOrigin_f(&utm);
#else
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &gps.ecef_pos);
  ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ltp_def);
#endif
  ins_mekf_wind_wrapper_Reset(true);
  //ins_mekf_wind.gps_initialized = false;
}

void ins_reset_altitude_ref(void)
{
#if MEKF_WIND_USE_UTM
  struct UtmCoor_f utm = state.utm_origin_f;
  utm.alt = gps.hmsl / 1000.0f;
  stateSetLocalUtmOrigin_f(&utm);
#else
  struct LlaCoor_i lla = {
    .lat = state.ned_origin_i.lla.lat,
    .lon = state.ned_origin_i.lla.lon,
    .alt = gps.lla_pos.alt
  };
  struct LtpDef_i ltp_def;
  ltp_def_from_lla_i(&ltp_def, &lla);
  ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ltp_def);
#endif
}

