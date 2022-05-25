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
 * @file modules/ahrs/ahrs_float_mlkf.h
 *
 * Multiplicative linearized Kalman Filter in quaternion formulation.
 *
 * Estimate the attitude, heading and gyro bias.
 */

#ifndef AHRS_FLOAT_MLKF_H
#define AHRS_FLOAT_MLKF_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

enum AhrsMlkfStatus {
  AHRS_MLKF_UNINIT,
  AHRS_MLKF_RUNNING
};

struct AhrsMlkf {
  struct FloatQuat   ltp_to_body_quat; ///< Rotation from LocalTangentPlane to body frame as unit quaternion
  struct FloatRates  body_rate;         ///< Rotational velocity in body frame
  struct FloatRates  gyro_bias;

  struct FloatVect3  mag_h;

  struct FloatVect3  mag_noise;

  struct FloatQuat  gibbs_cor;
  float P[6][6];
  float lp_accel;

  enum AhrsMlkfStatus status;
  bool is_aligned;
};

extern struct AhrsMlkf ahrs_mlkf;

extern void ahrs_mlkf_init(void);
extern bool ahrs_mlkf_align(struct FloatRates *lp_gyro, struct FloatVect3 *lp_accel,
                              struct FloatVect3 *lp_mag);
extern void ahrs_mlkf_propagate(struct FloatRates *gyro, float dt);
extern void ahrs_mlkf_update_accel(struct FloatVect3 *accel);
extern void ahrs_mlkf_update_mag(struct FloatVect3 *mag);
extern void ahrs_mlkf_update_mag_2d(struct FloatVect3 *mag);
extern void ahrs_mlkf_update_mag_full(struct FloatVect3 *mag);

#endif /* AHRS_FLOAT_MLKF_H */
