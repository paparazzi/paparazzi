/*
 * Copyright (C) 2011-2012  Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2013       Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/ahrs/ahrs_float_mlkf.h
 *
 * Multiplicative linearized Kalman Filter in quaternion formulation.
 *
 * Estimate the attitude, heading and gyro bias.
 */

#ifndef AHRS_FLOAT_MLKF_H
#define AHRS_FLOAT_MLKF_H

#include "subsystems/ahrs.h"
#include "std.h"
#include "math/pprz_algebra_float.h"

struct AhrsMlkf {
  struct FloatQuat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as unit quaternion
  struct FloatQuat   ltp_to_body_quat; ///< Rotation from LocalTangentPlane to body frame as unit quaternion
  struct FloatRates  imu_rate;         ///< Rotational velocity in IMU frame
  struct FloatRates  gyro_bias;

  struct FloatVect3  mag_h;

  struct FloatVect3  mag_noise;

  struct FloatQuat  gibbs_cor;
  float P[6][6];
  float lp_accel;
};

extern struct AhrsMlkf ahrs_impl;

#endif /* AHRS_FLOAT_MLKF_H */
