/*
 * Copyright (C) 2013 Michal Podhradsky
 * Utah State University, http://aggieair.usu.edu/
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
 * @file ahrs_aggienav.c
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "subsystems/ahrs/ahrs_aggienav.h"

#ifdef AHRS_UPDATE_FW_ESTIMATOR
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
#endif

/*
 * Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
 * Positive pitch : nose up
 * Positive roll : right wing down
 * Positive yaw : clockwise
 */
struct AhrsFloatQuat ahrs_impl;
struct AhrsAligner ahrs_aligner;

void ahrs_align(void) {
#ifdef USE_GX3
  imu_align();
#endif
}

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;
  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_impl.ltp_to_imu_quat, imuf.body_to_imu_quat);
#ifdef IMU_MAG_OFFSET
  ahrs_impl.mag_offset = IMU_MAG_OFFSET;
#else
  ahrs_impl.mag_offset = 0.0;
#endif
  ///ahrs_aligner.status = AHRS_ALIGNER_LOCKED;
  /// FIXME: Aligner not needed for GX3
  ahrs.status = AHRS_RUNNING;
}

void ahrs_aligner_run(void) {
#ifdef AHRS_ALIGNER_LED
  LED_ON(AHRS_ALIGNER_LED);
#endif
  ahrs.status = AHRS_RUNNING;
}


void ahrs_aligner_init(void) {
}

void ahrs_propagate(void) {
  // Rates
  static struct FloatRates body_rate;
  RATES_BFP_OF_REAL(imu.gyro, imuf.gyro ); // for backwards compatibility with fixed point interface
  // compute body rates
  FLOAT_RMAT_TRANSP_RATEMULT(body_rate, imuf.body_to_imu_rmat, imuf.gyro);
  // Set state
  stateSetBodyRates_f(&body_rate);

  // Attitude
  static struct FloatRMat ltp_to_body_rmat;
  FLOAT_RMAT_COMP(ltp_to_body_rmat, imu_gx3.rmat, imuf.body_to_imu_rmat);
#ifdef AHRS_UPDATE_FW_ESTIMATOR // fixedwing
  static struct FloatEulers ltp_to_body_eulers;
  FLOAT_EULERS_OF_RMAT(ltp_to_body_eulers, ltp_to_body_rmat);
  ltp_to_body_eulers.phi -= ins_roll_neutral;
  ltp_to_body_eulers.theta -= ins_pitch_neutral;
#if AHRS_USE_GPS_HEADING && USE_GPS
  static float course_f = (float)DegOfRad(gps.course / 1e7);
  if (course_f > 180.0) {
    course_f -= 360.0;
  }
  ltp_to_body_eulers.psi = (float)RadOfDeg(course_f);
#endif
  stateSetNedToBodyEulers_f(&ltp_to_body_eulers);
#else
#ifdef IMU_MAG_OFFSET //rotorcraft
  static struct FloatEulers ltp_to_body_eulers;
  FLOAT_EULERS_OF_RMAT(ltp_to_body_eulers, ltp_to_body_rmat);
  ltp_to_body_eulers.psi -= ahrs_impl.mag_offset;
  stateSetNedToBodyEulers_f(&ltp_to_body_eulers);
#else
  stateSetNedToBodyRMat_f(&ltp_to_body_rmat);
#endif
#endif
}

void ahrs_update_accel(void) {
}

void ahrs_update_mag(void) {
}

void ahrs_update_gps(void) {
}
