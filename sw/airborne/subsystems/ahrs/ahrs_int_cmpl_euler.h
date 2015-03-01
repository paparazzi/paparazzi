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

/**
 * @file subsystems/ahrs/ahrs_int_cmpl_euler.h
 *
 * Complementary filter in euler representation (fixed-point).
 *
 */


#ifndef AHRS_INT_CMPL_EULER_H
#define AHRS_INT_CMPL_EULER_H

#include "subsystems/ahrs.h"
#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_orientation_conversion.h"

enum AhrsICEStatus {
  AHRS_ICE_UNINIT,
  AHRS_ICE_RUNNING
};

struct AhrsIntCmplEuler {
  struct Int32Rates  gyro_bias;
  struct Int32Rates  imu_rate;
  struct Int32Eulers hi_res_euler;
  struct Int32Eulers measure;
  struct Int32Eulers residual;
  struct Int32Eulers measurement;
  struct Int32Eulers ltp_to_imu_euler;
  int32_t reinj_1;
  float mag_offset;

  struct OrientationReps body_to_imu;

  enum AhrsICEStatus status;
  bool_t is_aligned;
};

extern struct AhrsIntCmplEuler ahrs_ice;

extern void ahrs_ice_init(void);
extern void ahrs_ice_set_body_to_imu(struct OrientationReps *body_to_imu);
extern void ahrs_ice_set_body_to_imu_quat(struct FloatQuat *q_b2i);
extern bool_t ahrs_ice_align(struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                             struct Int32Vect3 *lp_mag);
extern void ahrs_ice_propagate(struct Int32Rates *gyro);
extern void ahrs_ice_update_accel(struct Int32Vect3 *accel);
extern void ahrs_ice_update_mag(struct Int32Vect3 *mag);

#endif /* AHRS_INT_CMPL_EULER_H */
