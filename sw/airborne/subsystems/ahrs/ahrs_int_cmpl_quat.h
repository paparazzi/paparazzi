/*
 * Copyright (C) 2008-2013 The Paparazzi Team
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
#include "modules/gps/gps.h"
#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_orientation_conversion.h"

enum AhrsICQStatus {
  AHRS_ICQ_UNINIT,
  AHRS_ICQ_RUNNING
};

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
  struct Int32Vect3  mag_h;

  int32_t ltp_vel_norm;
  bool ltp_vel_norm_valid;
  bool heading_aligned;
  float weight;
  float accel_inv_kp;
  float accel_inv_ki;
  float mag_kp;
  float mag_ki;

  /* parameters/options that can be changed */
  /** enable gravity vector correction by removing centrifugal acceleration */
  bool correct_gravity;

  /** sets how strongly the gravity heuristic reduces accel correction.
   * Set to zero in order to disable gravity heuristic.
   */
  uint8_t gravity_heuristic_factor;

  /** filter cut-off frequency for correcting the attitude from accels.
   *  (pseudo-gravity measurement)
   * only update through #ahrs_int_cmpl_quat_SetAccelOmega
   */
  float accel_omega;

  /** filter damping for correcting the gyro-bias from accels.
   *  (pseudo-gravity measurement)
   * only update through #ahrs_int_cmpl_quat_SetAccelZeta
   */
  float accel_zeta;

  /** filter cut-off frequency for correcting the attitude (heading) from magnetometer.
   * only update through #ahrs_int_cmpl_quat_SetMagOmega
   */
  float mag_omega;

  /** filter damping for correcting the gyro bias from magnetometer.
   * only update through #ahrs_int_cmpl_quat_SetMagZeta
   */
  float mag_zeta;

  /* internal counters for the gains */
  uint16_t accel_cnt; ///< number of propagations since last accel update
  uint16_t mag_cnt;   ///< number of propagations since last mag update

  struct OrientationReps body_to_imu;

  enum AhrsICQStatus status; ///< status of the AHRS, AHRS_ICQ_UNINIT or AHRS_ICQ_RUNNING
  bool is_aligned;
};

extern struct AhrsIntCmplQuat ahrs_icq;

extern void ahrs_icq_init(void);
extern void ahrs_icq_set_body_to_imu(struct OrientationReps *body_to_imu);
extern void ahrs_icq_set_body_to_imu_quat(struct FloatQuat *q_b2i);
extern bool ahrs_icq_align(struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                             struct Int32Vect3 *lp_mag);
extern void ahrs_icq_propagate(struct Int32Rates *gyro, float dt);
extern void ahrs_icq_update_accel(struct Int32Vect3 *accel, float dt);
extern void ahrs_icq_update_mag(struct Int32Vect3 *mag, float dt);
extern void ahrs_icq_update_gps(struct GpsState *gps_s);

/** Update yaw based on a heading measurement.
 * e.g. from GPS course
 * @param heading Heading in body frame, radians (CW/north) with #INT32_ANGLE_FRAC
 */
void ahrs_icq_update_heading(int32_t heading);

/** Hard reset yaw to a heading.
 * Doesn't affect the bias.
 * Sets ahrs_icq.heading_aligned to TRUE.
 * @param heading Heading in body frame, radians (CW/north) with #INT32_ANGLE_FRAC
 */
void ahrs_icq_realign_heading(int32_t heading);


/// update pre-computed inv_kp and inv_ki gains from acc_omega and acc_zeta
extern void ahrs_icq_set_accel_gains(void);

static inline void ahrs_int_cmpl_quat_SetAccelOmega(float omega)
{
  ahrs_icq.accel_omega = omega;
  ahrs_icq_set_accel_gains();
}

static inline void ahrs_int_cmpl_quat_SetAccelZeta(float zeta)
{
  ahrs_icq.accel_zeta = zeta;
  ahrs_icq_set_accel_gains();
}

/// update pre-computed kp and ki gains from mag_omega and mag_zeta
extern void ahrs_icq_set_mag_gains(void);

static inline void ahrs_int_cmpl_quat_SetMagOmega(float omega)
{
  ahrs_icq.mag_omega = omega;
  ahrs_icq_set_mag_gains();
}

static inline void ahrs_int_cmpl_quat_SetMagZeta(float zeta)
{
  ahrs_icq.mag_zeta = zeta;
  ahrs_icq_set_mag_gains();
}


#endif /* AHRS_INT_CMPL_QUAT_H */
