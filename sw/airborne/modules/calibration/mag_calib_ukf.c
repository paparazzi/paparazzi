/*
 * Copyright (C) w.vlenterie
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
 * @file "modules/calibration/mag_calib_ukf.c"
 * @author w.vlenterie
 * Calibrate the magnetometer using an unscented kalman filter
 * For more information please visit the following links:
 *   - https://github.com/sfwa/trical
 *   - http://au.tono.my/log/20131213-trical-magnetometer-calibration.html
 *   - http://www.acsu.buffalo.edu/~johnc/mag_cal05.pdf
 */

#include <stdio.h>
#include <error.h>
#include <stdbool.h>
#include <time.h>

#include "modules/calibration/mag_calib_ukf.h"
#include "modules/geo_mag/geo_mag.h"                     ///< The geo_mag module doesn't support requesting for updates so we need it's structure
#include "subsystems/ahrs/ahrs_magnetic_field_model.h"
#include "math/pprz_algebra_double.h"
#include "TRICAL.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/gps.h"
#include "subsystems/abi_common.h"
#include "abi_messages.h"

static void mag_calib_ukf_run(uint8_t __attribute__((unused)) sender_id, uint32_t __attribute__((unused)) stamp, struct Int32Vect3 *mag);
static void mag_calib_update_field(uint8_t __attribute__((unused)) sender_id, struct FloatVect3 *h);

#define PRINT(string,...) fprintf(stderr, "[CALIB_UKF->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if !defined MAG_CALIB_UKF_ABI_BIND_ID
#define MAG_CALIB_UKF_ABI_BIND_ID IMU_BOARD_ID
#endif

#if !defined MAG_CALIB_UKF_GEO_MAG_TIMEOUT
#define MAG_CALIB_UKF_GEO_MAG_TIMEOUT 0
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_GEO_MAG_TIMEOUT)

#if !defined MAG_CALIB_UKF_NORM
#define MAG_CALIB_UKF_NORM 1.0f
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_NORM)

#if !defined MAG_CALIB_UKF_NOISE_RMS
#define MAG_CALIB_UKF_NOISE_RMS 2e-1f
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_NOISE_RMS)

#if !defined MAG_CALIB_UKF_HOTSTART
#define MAG_CALIB_UKF_HOTSTART FALSE
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_HOTSTART)

#if !defined MAG_CALIB_UKF_HOTSTART_SAVE_FILE
#define MAG_CALIB_UKF_HOTSTART_SAVE_FILE /data/ftp/internal_000/mag_ukf_calib.txt
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_HOTSTART_SAVE_FILE)

#if !defined MAG_CALIB_UKF_VERBOSE || !MAG_CALIB_UKF_VERBOSE
#define VERBOSE_PRINT(...)
#else
#define VERBOSE_PRINT PRINT
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_VERBOSE)

bool settings_reset_state = false;

static TRICAL_instance_t mag_calib;
static abi_event mag_ev;
static abi_event h_ev;

static uint32_t timestamp_geo_mag;
static struct FloatVect3 H = { .x = AHRS_H_X, .y = AHRS_H_Y, .z =  AHRS_H_Z};

#if MAG_CALIB_UKF_HOTSTART
static FILE *fp;
static char hotstart_file_name[512];
#endif

void mag_calib_ukf_init(void)
{
#if USE_MAGNETOMETER
  TRICAL_init(&mag_calib);
  TRICAL_norm_set(&mag_calib, MAG_CALIB_UKF_NORM);
  TRICAL_noise_set(&mag_calib, MAG_CALIB_UKF_NOISE_RMS);

  mag_calib_hotstart_read();
  AbiBindMsgIMU_MAG_INT32(IMU_BOARD_ID, &mag_ev, mag_calib_ukf_run);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &h_ev, mag_calib_update_field);    ///< GEO_MAG_SENDER_ID is defined in geo_mag.c so unknown
#endif
}

void mag_calib_ukf_run(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag)
{
  float measurement[3] = {0.0, 0.0, 0.0}, calibrated_measurement[3] = {0.0, 0.0, 0.0};
  if (sender_id != MAG_CALIB_UKF_ID) {
    /** Update geo_mag based on MAG_CALIB_UKF_GEO_MAG_TIMEOUT (0 = no periodic updates) **/
    if (MAG_CALIB_UKF_GEO_MAG_TIMEOUT && GpsFixValid() && (get_sys_time_msec() - timestamp_geo_mag) >= 1000 * MAG_CALIB_UKF_GEO_MAG_TIMEOUT) {
      geo_mag.ready     = false;
      geo_mag.calc_once = true;   ///< Geo_mag will not re-update the calculation when the throttle is on so this is neccesary
    }
    /** See if we need to reset the state **/
    if (settings_reset_state) {
      TRICAL_reset(&mag_calib);
      settings_reset_state = false;
    }
    /** Update magnetometer UKF and calibrate measurement **/
    if (mag->x != 0 || mag->y != 0 || mag->z != 0) {
      /** Rotate the local magnetic field by our current attitude **/
      struct FloatQuat *body_quat = stateGetNedToBodyQuat_f();
      struct FloatVect3 expected_measurement;
      float_quat_vmult(&expected_measurement, body_quat, &H);
      float expected_mag_field[3] = { expected_measurement.x, expected_measurement.y, expected_measurement.z };
      /** Update magnetometer UKF **/
      measurement[0] = MAG_FLOAT_OF_BFP(mag->x);
      measurement[1] = MAG_FLOAT_OF_BFP(mag->y);
      measurement[2] = MAG_FLOAT_OF_BFP(mag->z);
      TRICAL_estimate_update(&mag_calib, measurement, expected_mag_field);
      TRICAL_measurement_calibrate(&mag_calib, measurement, calibrated_measurement);
      /** Save calibrated result **/
      mag->x = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[0]);
      mag->y = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[1]);
      mag->z = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[2]);
      VERBOSE_PRINT("magnetometer measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", measurement[0], measurement[1], measurement[2], hypot(hypot(measurement[0], measurement[1]), measurement[2]));
      VERBOSE_PRINT("magnetometer bias_f      (x: %4.2f  y: %4.2f  z: %4.2f)\n", mag_calib.state[0], mag_calib.state[1],  mag_calib.state[2]);
      VERBOSE_PRINT("expected measurement     (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", expected_mag_field[0], expected_mag_field[1], expected_mag_field[2], hypot(hypot(expected_mag_field[0], expected_mag_field[1]), expected_mag_field[2]));
      VERBOSE_PRINT("calibrated   measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n\n", calibrated_measurement[0], calibrated_measurement[1], calibrated_measurement[2], hypot(hypot(calibrated_measurement[0], calibrated_measurement[1]), calibrated_measurement[2]));
    }
    AbiSendMsgIMU_MAG_INT32(MAG_CALIB_UKF_ID, stamp, mag);
  }
}

void mag_calib_update_field(uint8_t __attribute__((unused)) sender_id, struct FloatVect3 *h)
{
  double n                = float_vect3_norm(h);
  if (n > 0.01) {
    H.x               = (float) h->x / n;
    H.y               = (float) h->y / n;
    H.z               = (float) h->z / n;
    timestamp_geo_mag = get_sys_time_msec();
    VERBOSE_PRINT("Updating local magnetic field from geo_mag module (Hx: %4.2f, Hy: %4.2f, Hz: %4.2f)\n", H.x, H.y, H.z);
  }
}

void mag_calib_hotstart_read(void)
{
#if MAG_CALIB_UKF_HOTSTART
  snprintf(hotstart_file_name, 512, "%s", STRINGIFY(MAG_CALIB_UKF_HOTSTART_SAVE_FILE));
  fp = fopen(hotstart_file_name, "r");
  if (fp != NULL) {
    fread(mag_calib.state, sizeof(float), 12, fp);
    fclose(fp);
    VERBOSE_PRINT("Loaded initial state from disk:\n"
                  "bias  {%4.2f, %4.2f, %4.2f}\n"
                  "scale {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n",
                  mag_calib.state[0], mag_calib.state[1],  mag_calib.state[2],
                  mag_calib.state[3], mag_calib.state[4],  mag_calib.state[5],
                  mag_calib.state[6], mag_calib.state[7],  mag_calib.state[8],
                  mag_calib.state[9], mag_calib.state[10], mag_calib.state[11]
                 );
  }
#endif
}

void mag_calib_hotstart_write(void)
{
#if USE_MAGNETOMETER && MAG_CALIB_UKF_HOTSTART
  fp = fopen(hotstart_file_name, "w");
  if (fp != NULL) {
    fwrite(mag_calib.state, sizeof(float), 12, fp);
    fclose(fp);
    VERBOSE_PRINT("Wrote current state to disk:\n"
                  "bias  {%4.2f, %4.2f, %4.2f}\n"
                  "scale {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n",
                  mag_calib.state[0], mag_calib.state[1],  mag_calib.state[2],
                  mag_calib.state[3], mag_calib.state[4],  mag_calib.state[5],
                  mag_calib.state[6], mag_calib.state[7],  mag_calib.state[8],
                  mag_calib.state[9], mag_calib.state[10], mag_calib.state[11]
                 );
  }
#endif
}
