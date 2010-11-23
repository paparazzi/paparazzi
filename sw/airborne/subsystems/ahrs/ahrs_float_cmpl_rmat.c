/*
 * $Id$
 *
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

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_float_cmpl_rmat.h"
#include "subsystems/ahrs/ahrs_float_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/imu.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_simple_matrix.h"
#include "generated/airframe.h"

#include <string.h>

#include "../../test/pprz_algebra_print.h"


static inline void compute_imu_quat_and_euler_from_rmat(void);
static inline void compute_body_orientation_and_rates(void);


struct AhrsFloatCmplRmat ahrs_impl;

void ahrs_init(void) {
  ahrs_float.status = AHRS_UNINIT;

  /*
   * Initialises our IMU alignement variables
   * This should probably done in the IMU code instead
   */
  struct FloatEulers body_to_imu_euler =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  FLOAT_QUAT_OF_EULERS(ahrs_impl.body_to_imu_quat, body_to_imu_euler);
  FLOAT_RMAT_OF_EULERS(ahrs_impl.body_to_imu_rmat, body_to_imu_euler);

  /* set ltp_to_body to zero */
  FLOAT_QUAT_ZERO(ahrs_float.ltp_to_body_quat);
  FLOAT_EULERS_ZERO(ahrs_float.ltp_to_body_euler);
  FLOAT_RMAT_ZERO(ahrs_float.ltp_to_body_rmat);
  FLOAT_RATES_ZERO(ahrs_float.body_rate);

  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_float.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  RMAT_COPY(ahrs_float.ltp_to_imu_rmat, ahrs_impl.body_to_imu_rmat);
  EULERS_COPY(ahrs_float.ltp_to_imu_euler, body_to_imu_euler);
  FLOAT_RATES_ZERO(ahrs_float.imu_rate);

}


void ahrs_align(void) {

  /* Compute an initial orientation using euler angles */
  ahrs_float_get_euler_from_accel_mag(&ahrs_float.ltp_to_imu_euler, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);
  /* Convert initial orientation in quaternion and rotation matrice representations. */
  FLOAT_QUAT_OF_EULERS(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_euler);
  FLOAT_RMAT_OF_QUAT(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_quat);
  /* Compute initial body orientation */
  compute_body_orientation_and_rates();

  /* used averaged gyro as initial value for bias */
  struct Int32Rates bias0;
  RATES_COPY(bias0, ahrs_aligner.lp_gyro);
  RATES_FLOAT_OF_BFP(ahrs_impl.gyro_bias, bias0);

  ahrs.status = AHRS_RUNNING;

}


void ahrs_propagate(void) {

  /* converts gyro to floating point */
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro_prev);
  /* unbias measurement */
  RATES_DIFF(ahrs_float.imu_rate, gyro_float, ahrs_impl.gyro_bias);
  const float dt = 1./512.;
  /* add correction     */
  struct FloatRates omega;
  RATES_SUM(omega, ahrs_float.imu_rate, ahrs_impl.rate_correction);
  //  DISPLAY_FLOAT_RATES("omega ", omega);
  /* and zeros it */
  FLOAT_RATES_ZERO(ahrs_impl.rate_correction);

  /* first order integration of rotation matrix */
  struct FloatRMat exp_omega_dt = {
    { 1.        ,  dt*omega.r, -dt*omega.q,
     -dt*omega.r,  1.        ,  dt*omega.p,
      dt*omega.q, -dt*omega.p,  1.                       }};
  struct FloatRMat R_tdt;
  FLOAT_RMAT_COMP(R_tdt, ahrs_float.ltp_to_imu_rmat, exp_omega_dt);
  memcpy(&ahrs_float.ltp_to_imu_rmat, &R_tdt, sizeof(R_tdt));

  float_rmat_reorthogonalize(&ahrs_float.ltp_to_imu_rmat);
  //  struct FloatRMat test;
  //  FLOAT_RMAT_COMP_INV(test, ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_rmat);
  //  DISPLAY_FLOAT_RMAT("foo", test);

  compute_imu_quat_and_euler_from_rmat();
  compute_body_orientation_and_rates();

}

void ahrs_update_accel(void) {

  struct FloatVect3 accel_float;
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);
  struct FloatVect3* r2 = (struct FloatVect3*)(&RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,0));
  struct FloatVect3 residual;
  FLOAT_VECT3_CROSS_PRODUCT(residual, accel_float, (*r2));
  /* heuristic on acceleration norm */
  const float acc_norm = FLOAT_VECT3_NORM(accel_float);
  const float weight = Chop(1.-2*fabs(1-acc_norm/9.81), 0., 1.);
  //const float weight = 1.;
  /* compute correction */
  const float gravity_rate_update_gain = 5e-2;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual, weight*gravity_rate_update_gain);
  const float gravity_bias_update_gain = -5e-6;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual, weight*gravity_bias_update_gain);
  /* FIXME: saturate bias */

}

void ahrs_update_mag(void) {

  /* project mag on local tangeant plane */
  struct FloatVect3 magf;
  MAGS_FLOAT_OF_BFP(magf, imu.mag);
  const float cphi   = cosf(ahrs_float.ltp_to_imu_euler.phi);
  const float sphi   = sinf(ahrs_float.ltp_to_imu_euler.phi);
  const float ctheta = cosf(ahrs_float.ltp_to_imu_euler.theta);
  const float stheta = sinf(ahrs_float.ltp_to_imu_euler.theta);
  const float mn = ctheta * magf.x + sphi*stheta*magf.y + cphi*stheta*magf.z;
  const float me =     0. * magf.x + cphi       *magf.y - sphi       *magf.z;

  const float res_norm = -RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 0,0)*me + RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 1,0)*mn;
  printf("res norm %f\n", res_norm);
  struct FloatVect3* r2 = (struct FloatVect3*)(&RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,0));
  DISPLAY_FLOAT_VECT3("r2", (*r2));
  const float mag_rate_update_gain = 2.5;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, (*r2), (mag_rate_update_gain*res_norm));
  DISPLAY_FLOAT_RATES("corr", ahrs_impl.rate_correction);
  const float mag_bias_update_gain = -2.5e-4;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, (*r2), (mag_bias_update_gain*res_norm));
  /* FIXME: saturate bias */

}

void ahrs_update_mag2(void) {

  const struct FloatVect3 expected_ltp = {AHRS_H_X, AHRS_H_Y, AHRS_H_Z};
  struct FloatVect3 expected_imu;
  FLOAT_RMAT_VECT3_MUL(expected_imu, ahrs_float.ltp_to_imu_rmat, expected_ltp);

  struct FloatVect3 measured_imu;
  MAGS_FLOAT_OF_BFP(measured_imu, imu.mag);

  struct FloatVect3 residual;
  FLOAT_VECT3_CROSS_PRODUCT(residual, measured_imu, expected_imu);
  // FLOAT_VECT3_DIFF(residual, expected_imu, measured_imu);

  DISPLAY_FLOAT_VECT3(" expected", expected_imu);
  DISPLAY_FLOAT_VECT3(" measured", measured_imu);
  DISPLAY_FLOAT_VECT3("residual", residual);

  const float mag_rate_update_gain = 2.5;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual, mag_rate_update_gain);

}

/*
 * Compute ltp to imu rotation in euler angles and quaternion representations
 * from the rotation matrice representation
 */
static inline void compute_imu_quat_and_euler_from_rmat(void) {
  FLOAT_QUAT_OF_RMAT(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_rmat);
  FLOAT_EULERS_OF_RMAT(ahrs_float.ltp_to_imu_euler, ahrs_float.ltp_to_imu_rmat);
}

/*
 * Compute body orientation and rates from imu orientation and rates
 */
static inline void compute_body_orientation_and_rates(void) {

  FLOAT_QUAT_COMP_INV(ahrs_float.ltp_to_body_quat,
		      ahrs_float.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  FLOAT_RMAT_COMP_INV(ahrs_float.ltp_to_body_rmat,
		      ahrs_float.ltp_to_imu_rmat, ahrs_impl.body_to_imu_rmat);
  FLOAT_EULERS_OF_RMAT(ahrs_float.ltp_to_body_euler, ahrs_float.ltp_to_body_rmat);
  FLOAT_RMAT_TRANSP_RATEMULT(ahrs_float.body_rate, ahrs_impl.body_to_imu_rmat, ahrs_float.imu_rate);

}
