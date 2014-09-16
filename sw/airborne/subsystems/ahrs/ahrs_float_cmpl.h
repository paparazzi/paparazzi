/*
 * Copyright (C) 2010 The Paparazzi Team
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
 * @file subsystems/ahrs/ahrs_float_cmpl.h
 *
 * Complementary filter in float to estimate the attitude, heading and gyro bias.
 *
 * Propagation can be done in rotation matrix or quaternion representation.
 */

#ifndef AHRS_FLOAT_CMPL
#define AHRS_FLOAT_CMPL

#include "std.h"

struct AhrsFloatCmpl {
  struct FloatRates gyro_bias;
  struct FloatRates rate_correction;
  struct FloatRates imu_rate;
  struct FloatQuat ltp_to_imu_quat;
  struct FloatRMat ltp_to_imu_rmat;

  bool_t correct_gravity; ///< enable gravity correction during coordinated turns
  float ltp_vel_norm; ///< velocity norm for gravity correction during coordinated turns
  bool_t ltp_vel_norm_valid;

  float accel_omega;  ///< filter cut-off frequency for correcting the attitude from accels (pseudo-gravity measurement)
  float accel_zeta;   ///< filter damping for correcting the gyro-bias from accels (pseudo-gravity measurement)
  float mag_omega;    ///< filter cut-off frequency for correcting the attitude (heading) from magnetometer
  float mag_zeta;     ///< filter damping for correcting the gyro bias from magnetometer

  /** sets how strongly the gravity heuristic reduces accel correction.
   * Set to zero in order to disable gravity heuristic.
   */
  uint8_t gravity_heuristic_factor;
  float weight;

  bool_t heading_aligned;
  struct FloatVect3 mag_h;

  /* internal counters for the gains */
  uint16_t accel_cnt; ///< number of propagations since last accel update
  uint16_t mag_cnt;   ///< number of propagations since last mag update
};

extern struct AhrsFloatCmpl ahrs_impl;


/** Update yaw based on a heading measurement.
 * e.g. from GPS course
 * @param heading Heading in body frame, radians (CW/north)
 */
void ahrs_update_heading(float heading);

/** Hard reset yaw to a heading.
 * Doesn't affect the bias.
 * Sets ahrs_impl.heading_aligned to TRUE.
 * @param heading Heading in body frame, radians (CW/north)
 */
void ahrs_realign_heading(float heading);


#endif /* AHRS_FLOAT_CMPL_RMAT */
