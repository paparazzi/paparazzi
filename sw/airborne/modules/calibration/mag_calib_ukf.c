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
#include "subsystems/ahrs/ahrs_magnetic_field_model.h"
#include "math/pprz_algebra_double.h"
#include "TRICAL.h"
#include "state.h"
#include "subsystems/imu/imu_bebop.h"
#include "modules/geo_mag/geo_mag.h"
#include "generated/airframe.h"
#include "subsystems/gps.h"
#include "subsystems/abi_common.h"
#include "abi_messages.h"

static void mag_calib_ukf_run(uint8_t __attribute__((unused)) sender_id, uint32_t __attribute__((unused)) stamp, struct Int32Vect3 *mag);

#define PRINT(string,...) fprintf(stderr, "[CALIB_UKF->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

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
#define MAG_CALIB_UKF_HOTSTART TRUE
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

TRICAL_instance_t mag_calib;
static abi_event mag_ev;

time_t mag_field_from_geo_mag;
bool   update_geo_mag_field = true;
struct FloatVect3 H = { .x = MAG_CALIB_UKF_NORM, .y = 0.0f, .z =  0.0f};

#if MAG_CALIB_UKF_HOTSTART
static FILE *fp;
char hotstart_file_name[512];
#endif

void mag_calib_ukf_init(void)
{
#if USE_MAGNETOMETER
  TRICAL_init(&mag_calib);
  TRICAL_norm_set(&mag_calib, MAG_CALIB_UKF_NORM);
  TRICAL_noise_set(&mag_calib, MAG_CALIB_UKF_NOISE_RMS);

  H.x = AHRS_H_X;
  H.y = AHRS_H_Y;
  H.z = AHRS_H_Z;
  VERBOSE_PRINT("Local magnetic field loaded from airframe file (Hx: %4.2f, Hy: %4.2f, Hz: %4.2f)\n", H.x, H.y, H.z);

  mag_calib_hotstart_read();
  AbiBindMsgIMU_MAG_INT32(IMU_BOARD_ID, &mag_ev, mag_calib_ukf_run);
#endif
}

void mag_calib_ukf_run(uint8_t __attribute__((unused)) sender_id, uint32_t __attribute__((unused)) stamp, struct Int32Vect3 *mag)
{
  float measurement[3] = {0.0, 0.0, 0.0}, calibrated_measurement[3] = {0.0, 0.0, 0.0};
  /** Update geo_mag based on MAG_CALIB_UKF_GEO_MAG_TIMEOUT (0 = no periodic updates) **/
  if (update_geo_mag_field && geo_mag.ready) {
    double n                = double_vect3_norm(&geo_mag.vect);
    if (n > 0.01) {
      H.x                     = (float) geo_mag.vect.x / n;
      H.y                     = (float) geo_mag.vect.y / n;
      H.z                     = (float) geo_mag.vect.z / n;
      mag_field_from_geo_mag  = time(0);
      update_geo_mag_field    = false;
      VERBOSE_PRINT("Updating local magnetic field from geo_mag module (Hx: %4.2f, Hy: %4.2f, Hz: %4.2f)\n", H.x, H.y, H.z);
    }
  }
  if (MAG_CALIB_UKF_GEO_MAG_TIMEOUT && GpsFixValid() && difftime(time(0), mag_field_from_geo_mag) >= MAG_CALIB_UKF_GEO_MAG_TIMEOUT) {
    geo_mag.ready           = false;
    geo_mag.calc_once       = true;   ///< Geo_mag will not re-update the calculation when the throttle is on so this is neccesary
    update_geo_mag_field    = true;
  }
  /** Update magnetometer UKF and calibrate measurement **/
  if (mag->x != 0 || mag->y != 0 || mag->z != 0) {
    /** Rotate the local magnetic field by our current attitude **/
    struct FloatQuat *body_quat = stateGetNedToBodyQuat_f();
    struct FloatRMat body_rmat;
    float_rmat_of_quat(&body_rmat, body_quat);
    struct FloatVect3 expected_measurement;
    float_rmat_vmult(&expected_measurement, &body_rmat, &H);
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
  AbiSendMsgIMU_MAG_INT32(MAG_CALIB_UKF_ID, get_sys_time_usec(), mag);
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
