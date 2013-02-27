/*
 * Copyright (C) 2011 The Paparazzi Team
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
 * @file subsystems/ahrs/ahrs_int_cmpl_quat.h
 *
 * Quaternion complementary filter (fixed-point).
 *
 * Estimate the attitude, heading and gyro bias.
 *
 */

#ifndef AHRS_INT_CMPL_QUAT_H
#define AHRS_INT_CMPL_QUAT_H

#include "subsystems/ahrs.h"
#include "std.h"
#include "math/pprz_algebra_int.h"

/**
 * Ahrs implementation specifc values
 */
struct AhrsIntCmplQuat {
  struct Int32Rates  gyro_bias;
  struct Int32Rates  imu_rate;
  struct Int32Rates  rate_correction;
  struct Int64Quat   high_rez_quat;
  struct Int64Rates  high_rez_bias;
  struct Int32Quat   ltp_to_imu_quat;
  struct Int32Eulers ltp_to_imu_euler; // FIXME to compile telemetry
  struct Int32Vect3  mag_h;
  uint32_t accel_attitude_gain;  ///< gain for correcting the attitude from accels (pseudo-gravity measurement)
  uint32_t accel_gyrobias_gain;  ///< gain for correcting the gyro-bias from accels (pseudo-gravity measurement)
  uint32_t mag_attitude_gain;    ///< gain for correcting the attitude (heading) from magnetometer
  uint32_t mag_gyrobias_gain;    ///< gain for correcting the gyro bias from magnetometer
  int32_t ltp_vel_norm;
  bool_t ltp_vel_norm_valid;
  bool_t correct_gravity;
  bool_t use_gravity_heuristic;
  bool_t heading_aligned;
};

extern struct AhrsIntCmplQuat ahrs_impl;


/** Update yaw based on a heading measurement.
 * e.g. from GPS course
 * @param heading Heading in body frame, radians (CW/north) with #INT32_ANGLE_FRAC
 */
void ahrs_update_heading(int32_t heading);

/** Hard reset yaw to a heading.
 * Doesn't affect the bias.
 * Sets ahrs_impl.heading_aligned to TRUE.
 * @param heading Heading in body frame, radians (CW/north) with #INT32_ANGLE_FRAC
 */
void ahrs_realign_heading(int32_t heading);

#ifdef AHRS_UPDATE_FW_ESTIMATOR
extern float ins_roll_neutral;
extern float ins_pitch_neutral;
#endif


#endif /* AHRS_INT_CMPL_QUAT_H */
