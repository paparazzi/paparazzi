/*
 * Copyright (C) 2014-2015 Jean-Philippe Condomines, Gautier Hattenberger
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
 * @file modules/ahrs/ahrs_float_invariant.h
 * AHRS using invariant filter.
 * For more information, please send an email to "jp.condomines@gmail.com"
 */

#ifndef AHRS_FLOAT_INVARIANT_H
#define AHRS_FLOAT_INVARIANT_H

#include "modules/ahrs/ahrs.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

/** Invariant filter state dimension
 */
#define INV_STATE_DIM 9

/** Invariant filter state
 */
struct inv_state  {
  struct FloatQuat quat;  ///< Estimated attitude (quaternion)
  struct FloatRates bias; ///< Estimated gyro biases
  float cs;               ///< Estimates magnetometer sensitivity
  float as;               ///< Estimated accelerometer sensitivity
};

/** Invariant filter measurement vector dimension
 */
#define INV_MEASURE_DIM 6

/** Invariant filter measurement vector
 */
struct inv_measures {
  struct FloatVect3 accel;    ///< Measured accelerometers
  struct FloatVect3 mag;      ///< Measured magnetic field
};

/** Invariant filter command vector dimension
 */
#define INV_COMMAND_DIM 3

/** Invariant filter command vector
 */
struct inv_command {
  struct FloatRates rates;  ///< Input gyro rates
};

/** Invariant filter correction gains
 */
struct inv_correction_gains {
  struct FloatVect3 LE;   ///< Correction gains on attitude
  struct FloatVect3 ME;   ///< Correction gains on gyro biases
  float NE;               ///< Correction gains on accel bias
  float OE;               ///< Correction gains on magnetometer sensitivity
};

/** Invariant filter tuning gains
 */
struct inv_gains {
  float lx; ///< Tuning parameter of accel and mag on attitude (longitudinal subsystem)
  float ly; ///< Tuning parameter of accel and mag on attitude (lateral subsystem)
  float lz; ///< Tuning parameter of accel and mag on attitude (heading subsystem)
  float mx; ///< Tuning parameter of accel and mag on gyro bias (longitudinal subsystem)
  float my; ///< Tuning parameter of accel and mag on gyro bias (lateral subsystem)
  float mz; ///< Tuning parameter of accel and mag on gyro bias (heading subsystem)
  float n;  ///< Tuning parameter of accel and mag on accel bias (scaling subsystem)
  float o;  ///< Tuning parameter of accel and mag on mag bias (scaling subsystem)
};

/** Invariant filter structure
 */
struct AhrsFloatInv {
  struct inv_state state;             ///< state vector
  struct inv_measures meas;           ///< measurement vector
  struct inv_command cmd;             ///< command vector
  struct inv_correction_gains corr;   ///< correction gains
  struct inv_gains gains;             ///< tuning gains

  bool reset;                       ///< flag to request reset/reinit the filter

  /** body_to_imu rotation */
  struct OrientationReps body_to_imu;

  struct FloatVect3 mag_h;
  bool is_aligned;
};

extern struct AhrsFloatInv ahrs_float_inv;

extern void ahrs_float_invariant_init(void);
extern void ahrs_float_inv_set_body_to_imu_quat(struct FloatQuat *q_b2i);
extern void ahrs_float_invariant_align(struct FloatRates *lp_gyro,
                                       struct FloatVect3 *lp_accel,
                                       struct FloatVect3 *lp_mag);
extern void ahrs_float_invariant_propagate(struct FloatRates* gyro, float dt);
extern void ahrs_float_invariant_update_accel(struct FloatVect3* accel);
extern void ahrs_float_invariant_update_mag(struct FloatVect3* mag);

#endif /* AHRS_FLOAT_INVARIANT_H */

