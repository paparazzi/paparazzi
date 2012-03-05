/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

/** \file ahrs.h
 * \brief Attitude and Heading Reference System interface
 */

#ifndef AHRS_H
#define AHRS_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#define AHRS_UNINIT  0
#define AHRS_RUNNING 1

/* underlying includes (needed for parameters) */
#ifdef AHRS_TYPE_H
#include AHRS_TYPE_H
#endif

/** Attitude and Heading Reference System state (fixed point version) */
struct Ahrs {

  struct Int32Quat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as unit quaternion
  struct Int32Eulers ltp_to_imu_euler; ///< Rotation from LocalTangentPlane to IMU frame as Euler angles
  struct Int32RMat   ltp_to_imu_rmat;  ///< Rotation from LocalTangentPlane to IMU frame as Rotation Matrix
  struct Int32Rates  imu_rate;         ///< Rotational velocity in IMU frame

  struct Int32Quat   ltp_to_body_quat;  ///< Rotation from LocalTangentPlane to body frame as unit quaternion
  struct Int32Eulers ltp_to_body_euler; ///< Rotation from LocalTangentPlane to body frame as Euler angles
  struct Int32RMat   ltp_to_body_rmat;  ///< Rotation from LocalTangentPlane to body frame as Rotation Matrix
  struct Int32Rates  body_rate;         ///< Rotational velocity in body frame

  uint8_t status; ///< status of the AHRS, AHRS_UNINIT or AHRS_RUNNING
};

/** Attitude and Heading Reference System state (floating point version) */
struct AhrsFloat {
  struct FloatQuat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as unit quaternion
  struct FloatEulers ltp_to_imu_euler; ///< Rotation from LocalTangentPlane to IMU frame as Euler angles
  struct FloatRMat   ltp_to_imu_rmat;  ///< Rotation from LocalTangentPlane to IMU frame as Rotation Matrix
  struct FloatRates  imu_rate;         ///< Rotational velocity in IMU frame
  struct FloatRates  imu_rate_previous;
  struct FloatRates  imu_rate_d;

  struct FloatQuat   ltp_to_body_quat;  ///< Rotation from LocalTangentPlane to body frame as unit quaternion
  struct FloatEulers ltp_to_body_euler; ///< Rotation from LocalTangentPlane to body frame as Euler angles
  struct FloatRMat   ltp_to_body_rmat;  ///< Rotation from LocalTangentPlane to body frame as Rotation Matrix
  struct FloatRates  body_rate;         ///< Rotational velocity in body frame
  struct FloatRates  body_rate_d;

  // always use status from fixed point ahrs struct for now
  //uint8_t status;
};

/** global AHRS state (fixed point version) */
extern struct Ahrs ahrs;
/** global AHRS state (floating point version) */
extern struct AhrsFloat ahrs_float;

extern float ahrs_mag_offset;

#define AHRS_FLOAT_OF_INT32() {						       \
    QUAT_FLOAT_OF_BFP(ahrs_float.ltp_to_body_quat, ahrs.ltp_to_body_quat);     \
    EULERS_FLOAT_OF_BFP(ahrs_float.ltp_to_body_euler, ahrs.ltp_to_body_euler); \
    RATES_FLOAT_OF_BFP(ahrs_float.body_rate, ahrs.body_rate);		       \
  }

#define AHRS_INT_OF_FLOAT() {                                                  \
    QUAT_BFP_OF_REAL(ahrs.ltp_to_body_quat, ahrs_float.ltp_to_body_quat);      \
    EULERS_BFP_OF_REAL(ahrs.ltp_to_body_euler, ahrs_float.ltp_to_body_euler);  \
    RMAT_BFP_OF_REAL(ahrs.ltp_to_body_rmat, ahrs_float.ltp_to_body_rmat);      \
    RATES_BFP_OF_REAL(ahrs.body_rate, ahrs_float.body_rate);                   \
  }

#define AHRS_IMU_INT_OF_FLOAT() {                                       \
    QUAT_BFP_OF_REAL(ahrs.ltp_to_imu_quat, ahrs_float.ltp_to_imu_quat); \
    EULERS_BFP_OF_REAL(ahrs.ltp_to_imu_euler, ahrs_float.ltp_to_imu_euler); \
    RMAT_BFP_OF_REAL(ahrs.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_rmat); \
    RATES_BFP_OF_REAL(ahrs.imu_rate, ahrs_float.imu_rate);            \
  }

/** AHRS initialization. Called at startup.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_init(void);

/** Aligns the AHRS. Called after ahrs_aligner has run to set initial attitude and biases.
 *  Must set the ahrs status to AHRS_RUNNING.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_align(void);

/** Propagation. Usually integrates the gyro rates to angles.
 *  Reads the global #imu data struct.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_propagate(void);

/** Update AHRS state with accerleration measurements.
 *  Reads the global #imu data struct.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_update_accel(void);

/** Update AHRS state with magnetometer measurements.
 *  Reads the global #imu data struct.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_update_mag(void);
extern void ahrs_update_gps(void);

#endif /* AHRS_H */
