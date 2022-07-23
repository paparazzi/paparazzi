/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/ahrs/ahrs_madgwick.h
 * AHRS using Madgwick implementation
 *
 * See:
 * https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#include "modules/ahrs/ahrs_madgwick.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"
#include "modules/ahrs/ahrs_float_utils.h"

#ifndef AHRS_MADGWICK_BETA
#define AHRS_MADGWICK_BETA 0.1f    // 2 * proportional gain
#endif


struct AhrsMadgwick ahrs_madgwick;


/* init state and measurements */
static inline void init_state(void)
{
  float_quat_identity(&ahrs_madgwick.quat);
  FLOAT_RATES_ZERO(ahrs_madgwick.bias);
  FLOAT_VECT3_ZERO(ahrs_madgwick.accel);
}

void ahrs_madgwick_init(void)
{
  // init state and measurements
  init_state();

  ahrs_madgwick.is_aligned = false;
  ahrs_madgwick.reset = false;
}

void ahrs_madgwick_align(struct FloatRates *lp_gyro, struct FloatVect3 *lp_accel)
{
  /* Compute an initial orientation from accel directly as quaternion */
  ahrs_float_get_quat_from_accel(&ahrs_madgwick.quat, lp_accel);
  /* use average gyro as initial value for bias */
  ahrs_madgwick.bias = *lp_gyro;
  // ins and ahrs are now running
  ahrs_madgwick.is_aligned = true;
}

void ahrs_madgwick_propagate(struct FloatRates* gyro, float dt)
{
  struct FloatQuat qdot;

  // realign all the filter if needed
  // a complete init cycle is required
  if (ahrs_madgwick.reset) {
    ahrs_madgwick.reset = false;
    ahrs_madgwick.is_aligned = false;
    init_state();
  }

  // unbias gyro
  RATES_DIFF(ahrs_madgwick.rates, *gyro, ahrs_madgwick.bias);

  // Rate of change of quaternion from gyroscope
  float_quat_derivative(&qdot, &ahrs_madgwick.rates, &ahrs_madgwick.quat);

  // compute accel norm
  float norm = float_vect3_norm(&ahrs_madgwick.accel);
  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (norm > 0.01f) {
    // Normalise accelerometer measurement
    // direction of accel is inverted to comply with filter original frame
    struct FloatVect3 a;
    VECT3_SDIV(a, ahrs_madgwick.accel, -norm);

    // Auxiliary variables to avoid repeated arithmetic
    const float q1 = ahrs_madgwick.quat.qx;
    const float q2 = ahrs_madgwick.quat.qy;
    const float q3 = ahrs_madgwick.quat.qz;
    const float _2q0 = 2.0f * ahrs_madgwick.quat.qi;
    const float _2q1 = 2.0f * ahrs_madgwick.quat.qx;
    const float _2q2 = 2.0f * ahrs_madgwick.quat.qy;
    const float _2q3 = 2.0f * ahrs_madgwick.quat.qz;
    const float _4q0 = 4.0f * ahrs_madgwick.quat.qi;
    const float _4q1 = 4.0f * ahrs_madgwick.quat.qx;
    const float _4q2 = 4.0f * ahrs_madgwick.quat.qy;
    const float _8q1 = 8.0f * ahrs_madgwick.quat.qx;
    const float _8q2 = 8.0f * ahrs_madgwick.quat.qy;
    const float q0q0 = ahrs_madgwick.quat.qi * ahrs_madgwick.quat.qi;
    const float q1q1 = ahrs_madgwick.quat.qx * ahrs_madgwick.quat.qx;
    const float q2q2 = ahrs_madgwick.quat.qy * ahrs_madgwick.quat.qy;
    const float q3q3 = ahrs_madgwick.quat.qz * ahrs_madgwick.quat.qz;

    // Gradient decent algorithm corrective step
    const float s0 = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
    const float s1 = _4q1 * q3q3 - _2q3 * a.x + 4.0f * q0q0 * q1 - _2q0 * a.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z;
    const float s2 = 4.0f * q0q0 * q2 + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z;
    const float s3 = 4.0f * q1q1 * q3 - _2q1 * a.x + 4.0f * q2q2 * q3 - _2q2 * a.y;
    const float beta_inv_grad_norm = AHRS_MADGWICK_BETA / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

    // Apply feedback step
    qdot.qi -= s0 * beta_inv_grad_norm;
    qdot.qx -= s1 * beta_inv_grad_norm;
    qdot.qy -= s2 * beta_inv_grad_norm;
    qdot.qz -= s3 * beta_inv_grad_norm;
  }

  // Integrate rate of change of quaternion to yield quaternion
  ahrs_madgwick.quat.qi += qdot.qi * dt;
  ahrs_madgwick.quat.qx += qdot.qx * dt;
  ahrs_madgwick.quat.qy += qdot.qy * dt;
  ahrs_madgwick.quat.qz += qdot.qz * dt;

  // Normalise quaternion
  float_quat_normalize(&ahrs_madgwick.quat);
}

void ahrs_madgwick_update_accel(struct FloatVect3* accel)
{
  ahrs_madgwick.accel = *accel;
}
