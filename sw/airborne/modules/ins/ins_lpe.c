/*
 * Copyright (C) 2016 Michal Podhradsky <michal.pohradsky@aggiemail.usu.edu>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file modules/ins/ins_lpe.c
 *
 * Local Position Estimator
 */
#include "modules/ins/ins_lpe.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"
#include "message_pragmas.h"

#include "state.h"

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

/*
 * ABI bindings
 */
/** baro */
#ifndef INS_LPE_BARO_ID
#if USE_BARO_BOARD
#define INS_LPE_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_LPE_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_LPE_BARO_ID)
abi_event ins_lpe_baro_ev;
static void baro_cb(uint8_t sender_id, float pressure);


/** IMU (accel, body_to_imu) */
#ifndef INS_LPE_IMU_ID
#define INS_LPE_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_LPE_IMU_ID)
static abi_event accel_ev;
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);


/** ABI binding for GPS data.
 * Used for GPS ABI messages.
 */
#ifndef INS_LPE_GPS_ID
#define INS_LPE_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_LPE_GPS_ID)
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);


/** ABI binding for VELOCITY_ESTIMATE.
 * Usually this is coming from opticflow.
 */
#ifndef INS_LPE_VEL_ID
#define INS_LPE_VEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_LPE_VEL_ID)
static abi_event vel_est_ev;
static void vel_est_cb(uint8_t sender_id, uint32_t stamp, float x, float y, float z, float noise);


/** ABI binding for Lidar data.
 * Used for AGL ABI message with LIDAR ID
 */
#ifndef INS_LPE_LIDAR_ID
#define INS_LPE_LIDAR_ID ABI_BROADCAST // TODO: trigger a warning?
#endif
PRINT_CONFIG_VAR(INS_LPE_LIDAR_ID)
abi_event lidar_ev;
static void lidar_cb(uint8_t sender_id, float distance);


/** ABI binding for Sonar data.
 * Used for AGL ABI message with SONAR ID
 */
#ifndef INS_LPE_SONAR_ID
#define INS_LPE_SONAR_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_LPE_SONAR_ID)
abi_event sonar_ev;
static void sonar_cb(uint8_t sender_id, float distance);


// Ins struct
struct InsLpe ins_lpe;


/**
 * Init function
 * - Initialize variables, populate Kalman filter
 * - Bind ABI messages
 * - Initialize coordinate system
 * - Bind telemetry messages
 */
void ins_lpe_init(void)
{


  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_ACCEL_INT32(INS_LPE_IMU_ID, &accel_ev, accel_cb); // accel
  AbiBindMsgGPS(INS_LPE_GPS_ID, &gps_ev, gps_cb); // GPS
  AbiBindMsgVELOCITY_ESTIMATE(INS_LPE_VEL_ID, &vel_est_ev, vel_est_cb); // Optical flow
  AbiBindMsgAGL(INS_LPE_SONAR_ID, &sonar_ev, sonar_cb); // sonar
  AbiBindMsgAGL(INS_LPE_LIDAR_ID, &lidar_ev, lidar_cb); // lidar
  AbiBindMsgBARO_ABS(INS_LPE_BARO_ID, &ins_lpe_baro_ev, baro_cb); // baro
}


/**
 * Periodic function
 * - perform checks (std, errors, reset etc...)
 * - check which sensors are available
 * - check for NaN in matrices
 * - predict
 *   - accel input
 *   - correction logic (bias)
 *   - propagate
 * - update for each sensor
 * - final sanity checks & publish results / update state interface
 */
void ins_lpe_periodic(void)
{

}


/**
 *  Baro callback
 *  Copy the pressure and timestamp
 *  Set the baro new data flag
 */
static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{

}


/**
 * Sonar callback
 * Copy the distance and the timestamp
 * Set the sonar new data flag
 */
static void sonar_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  // AGL message doesn't provide timestamp, so use current time
  ins_lpe.sonar.timestamp = get_sys_time_usec();

  // save (derotated and properly oriented value) and remove offset
  ins_lpe.sonar.agl = distance - ins_lpe.sonar.offset;

  // correct for min/max
  if (ins_lpe.sonar.agl > ins_lpe.sonar.max_distance) {
    ins_lpe.sonar.agl = ins_lpe.sonar.max_distance;
  }
  if (ins_lpe.sonar.agl < ins_lpe.sonar.min_distance) {
    ins_lpe.sonar.agl = ins_lpe.sonar.min_distance;
  }

  // update flag
  ins_lpe.sonar.data_available = TRUE;
}


/**
 * Lidar callback
 * Copy the distance and the timestamp
 * Set the lidar new data flag
 */
static void lidar_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{
  // AGL message doesn't provide timestamp, so use current time
  ins_lpe.lidar.timestamp = get_sys_time_usec();

  // save (derotated and properly oriented value) and remove offset
  ins_lpe.lidar.agl = distance - ins_lpe.lidar.offset;

  // correct for min/max
  if (ins_lpe.lidar.agl > ins_lpe.lidar.max_distance) {
    ins_lpe.lidar.agl = ins_lpe.lidar.max_distance;
  }
  if (ins_lpe.lidar.agl < ins_lpe.lidar.min_distance) {
    ins_lpe.lidar.agl = ins_lpe.lidar.min_distance;
  }

  // update flag
  ins_lpe.lidar.data_available = TRUE;
}


/**
 * Accel callback
 * Copy the accel and the timestamp
 * Make sure accel is in body frame / NED frame
 * Set the new accel data flag
 */
static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  // get timestamp [us]
  ins_lpe.accel.timestamp = stamp;

  // derotate
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&ins_lpe.accel.accel_meas_body, body_to_imu_rmat, accel);
  // stateSetAccelBody_i(&ins_lpe.accel.accel_meas_body); // if we use only one INS
  int32_rmat_transp_vmult(&ins_lpe.accel.accel_meas_ltp, stateGetNedToBodyRMat_i(), &ins_lpe.accel.accel_meas_body);

  // update flag
  ins_lpe.accel.data_available = TRUE;
}



/**
 * GPS callback
 * Copy the lat/long/alt and the timestamp
 * Set the new GPS data flag
 */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{

}


/**
 * Optical flow callback
 * Copy the flow estimates and the timestamp
 * Set the new optical flow data flag
 */
static void vel_est_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp,
                       float x, float y, float z,
                       float noise __attribute__((unused)))
{

}

