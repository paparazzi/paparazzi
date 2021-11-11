/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file modules/imu/imu.h
 * Inertial Measurement Unit interface.
 */

#ifndef IMU_H
#define IMU_H

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"
#include "generated/airframe.h"


/** abstract IMU interface providing fixed point interface  */
struct Imu {
  struct Int32Rates gyro;             ///< gyroscope measurements in rad/s in BFP with #INT32_RATE_FRAC
  struct Int32Vect3 accel;            ///< accelerometer measurements in m/s^2 in BFP with #INT32_ACCEL_FRAC
  struct Int32Vect3 mag;              ///< magnetometer measurements scaled to 1 in BFP with #INT32_MAG_FRAC
  struct Int32Rates gyro_prev;        ///< previous gyroscope measurements
  struct Int32Vect3 accel_prev;       ///< previous accelerometer measurements
  struct Int32Rates gyro_neutral;     ///< static gyroscope bias from calibration in raw/unscaled units
  struct Int32Vect3 accel_neutral;    ///< static accelerometer bias from calibration in raw/unscaled units
  struct Int32Vect3 mag_neutral;      ///< magnetometer neutral readings (bias) in raw/unscaled units
  struct Int32Rates gyro_unscaled;    ///< unscaled gyroscope measurements
  struct Int32Vect3 accel_unscaled;   ///< unscaled accelerometer measurements
  struct Int32Vect3 mag_unscaled;     ///< unscaled magnetometer measurements
  struct OrientationReps body_to_imu; ///< rotation from body to imu frame

  /** flag for adjusting body_to_imu via settings.
   * if FALSE, reset to airframe values, if TRUE set current roll/pitch
   */
  bool b2i_set_current;
};

/** global IMU state */
extern struct Imu imu;

/* underlying hardware */
#ifdef IMU_TYPE_H
#include IMU_TYPE_H
#endif

extern void imu_init(void);
extern void imu_SetBodyToImuPhi(float phi);
extern void imu_SetBodyToImuTheta(float theta);
extern void imu_SetBodyToImuPsi(float psi);
extern void imu_SetBodyToImuCurrent(float set);
extern void imu_ResetBodyToImu(float reset);

/* can be provided implementation */
extern void imu_scale_gyro(struct Imu *_imu);
extern void imu_scale_accel(struct Imu *_imu);
extern void imu_scale_mag(struct Imu *_imu);

#if !defined IMU_BODY_TO_IMU_PHI && !defined IMU_BODY_TO_IMU_THETA && !defined IMU_BODY_TO_IMU_PSI
#define IMU_BODY_TO_IMU_PHI   0
#define IMU_BODY_TO_IMU_THETA 0
#define IMU_BODY_TO_IMU_PSI   0
#endif

#if !defined IMU_GYRO_P_NEUTRAL && !defined IMU_GYRO_Q_NEUTRAL && !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif

#if !defined IMU_ACCEL_X_NEUTRAL && !defined IMU_ACCEL_Y_NEUTRAL && !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif

#if !defined IMU_MAG_X_NEUTRAL && !defined IMU_MAG_Y_NEUTRAL && !defined IMU_MAG_Z_NEUTRAL
#define IMU_MAG_X_NEUTRAL 0
#define IMU_MAG_Y_NEUTRAL 0
#define IMU_MAG_Z_NEUTRAL 0
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN 1
#define IMU_MAG_Y_SIGN 1
#define IMU_MAG_Z_SIGN 1
#endif


#endif /* IMU_H */
