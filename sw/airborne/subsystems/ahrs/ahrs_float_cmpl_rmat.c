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


#include "../../test/pprz_algebra_print.h"

void ahrs_update_mag_full(void);
void ahrs_update_mag_2d(void);
void ahrs_update_mag_2d_dumb(void);

static inline void compute_imu_quat_and_euler_from_rmat(void);
static inline void compute_imu_rmat_and_euler_from_quat(void);
static inline void compute_body_orientation_and_rates(void);


struct AhrsFloatCmplRmat ahrs_impl;

void ahrs_init(void) {
  ahrs_float.status = AHRS_UNINIT;

  /* Initialises IMU alignement */
  struct FloatEulers body_to_imu_euler =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  FLOAT_QUAT_OF_EULERS(ahrs_impl.body_to_imu_quat, body_to_imu_euler);
  FLOAT_RMAT_OF_EULERS(ahrs_impl.body_to_imu_rmat, body_to_imu_euler);
  
  /* Set ltp_to_body to zero */
  FLOAT_QUAT_ZERO(ahrs_float.ltp_to_body_quat);
  FLOAT_EULERS_ZERO(ahrs_float.ltp_to_body_euler);
  FLOAT_RMAT_ZERO(ahrs_float.ltp_to_body_rmat);
  FLOAT_RATES_ZERO(ahrs_float.body_rate);

  /* Set ltp_to_imu so that body is zero */
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
  RATES_SUB(gyro_float, ahrs_impl.gyro_bias);
  
#ifdef AHRS_PROPAGATE_LOW_PASS_RATES
  const float alpha = 0.1;
  FLOAT_RATES_LIN_CMB(ahrs_float.imu_rate, ahrs_float.imu_rate, (1.-alpha), gyro_float, alpha); 
#else
  RATES_COPY(ahrs_float.imu_rate,gyro_float); 
#endif

  /* add correction     */
  struct FloatRates omega;
  RATES_SUM(omega, gyro_float, ahrs_impl.rate_correction);
  /* and zeros it */
  FLOAT_RATES_ZERO(ahrs_impl.rate_correction);

  const float dt = 1./AHRS_PROPAGATE_FREQUENCY;
#ifdef AHRS_PROPAGATE_RMAT
  FLOAT_RMAT_INTEGRATE_FI(ahrs_float.ltp_to_imu_rmat, omega, dt );
  float_rmat_reorthogonalize(&ahrs_float.ltp_to_imu_rmat);
  compute_imu_quat_and_euler_from_rmat();
#endif
#ifdef AHRS_PROPAGATE_QUAT
  FLOAT_QUAT_INTEGRATE(ahrs_float.ltp_to_imu_quat, omega, dt);
  FLOAT_QUAT_NORMALISE(ahrs_float.ltp_to_imu_quat);
  compute_imu_rmat_and_euler_from_quat();
#endif
  compute_body_orientation_and_rates();

}

void ahrs_update_accel(void) {

  struct FloatVect3 accel_float;
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);

#ifdef AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  float v = FLOAT_VECT3_NORM(ahrs_impl.est_ltp_speed);
  accel_float.y -=  v * ahrs_float.imu_rate.r;
  accel_float.z -= -v * ahrs_float.imu_rate.q;
#endif

  struct FloatVect3  c2 = { RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 0,2),
			    RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 1,2),
			    RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,2)};
  struct FloatVect3 residual;
  FLOAT_VECT3_CROSS_PRODUCT(residual, accel_float, c2);
#ifdef AHRS_GRAVITY_UPDATE_NORM_HEURISTIC
  /* heuristic on acceleration norm */
  const float acc_norm = FLOAT_VECT3_NORM(accel_float);
  const float weight = Chop(1.-6*fabs((9.81-acc_norm)/9.81), 0., 1.);
#else
  const float weight = 1.;
#endif
  /* compute correction */
  const float gravity_rate_update_gain = -5e-2; // -5e-2
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual, weight*gravity_rate_update_gain);
#if 1
  const float gravity_bias_update_gain = 1e-5; // -5e-6
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual, weight*gravity_bias_update_gain);
#else
  const float alpha = 5e-4;
  FLOAT_RATES_SCALE(ahrs_impl.gyro_bias, 1.-alpha);
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual, alpha);
#endif
  /* FIXME: saturate bias */

}


void ahrs_update_mag(void) {
#ifdef AHRS_MAG_UPDATE_YAW_ONLY
  ahrs_update_mag_2d();
  //  ahrs_update_mag_2d_dumb();
#else
  ahrs_update_mag_full();
#endif
}

void ahrs_update_mag_full(void) {

  const struct FloatVect3 expected_ltp = {AHRS_H_X, AHRS_H_Y, AHRS_H_Z};
  struct FloatVect3 expected_imu;
  FLOAT_RMAT_VECT3_MUL(expected_imu, ahrs_float.ltp_to_imu_rmat, expected_ltp);

  struct FloatVect3 measured_imu;
  MAGS_FLOAT_OF_BFP(measured_imu, imu.mag);

  struct FloatVect3 residual_imu;
  FLOAT_VECT3_CROSS_PRODUCT(residual_imu, measured_imu, expected_imu);
  //  DISPLAY_FLOAT_VECT3("# expected", expected_imu);
  //  DISPLAY_FLOAT_VECT3("# measured", measured_imu);
  //  DISPLAY_FLOAT_VECT3("# residual", residual);

  const float mag_rate_update_gain = 2.5;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual_imu, mag_rate_update_gain);

  const float mag_bias_update_gain = -2.5e-3;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual_imu, mag_bias_update_gain);

}

void ahrs_update_mag_2d(void) {

  const struct FloatVect2 expected_ltp = {AHRS_H_X, AHRS_H_Y};

  struct FloatVect3 measured_imu;
  MAGS_FLOAT_OF_BFP(measured_imu, imu.mag);
  struct FloatVect3 measured_ltp;
  FLOAT_RMAT_VECT3_TRANSP_MUL(measured_ltp, ahrs_float.ltp_to_imu_rmat, measured_imu);
   
  const struct FloatVect3 residual_ltp = 
    { 0,
      0,
      measured_ltp.x * expected_ltp.y - measured_ltp.y * expected_ltp.x };

  //  printf("res : %f\n", residual_ltp.z);

  struct FloatVect3 residual_imu;
  FLOAT_RMAT_VECT3_MUL(residual_imu, ahrs_float.ltp_to_imu_rmat, residual_ltp);
 
  const float mag_rate_update_gain = 2.5;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual_imu, mag_rate_update_gain);

  const float mag_bias_update_gain = -2.5e-3;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual_imu, mag_bias_update_gain);

}


void ahrs_update_mag_full_2d_dumb(void) {

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
  struct FloatVect3* r2 = (struct FloatVect3*)(&RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,0));
  const float mag_rate_update_gain = 2.5;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, (*r2), (mag_rate_update_gain*res_norm));
  const float mag_bias_update_gain = -2.5e-4;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, (*r2), (mag_bias_update_gain*res_norm));

}







/*
 * Compute ltp to imu rotation in euler angles and quaternion representations
 * from the rotation matrice representation
 */
static inline void compute_imu_quat_and_euler_from_rmat(void) {
  FLOAT_QUAT_OF_RMAT(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_rmat);
  FLOAT_EULERS_OF_RMAT(ahrs_float.ltp_to_imu_euler, ahrs_float.ltp_to_imu_rmat);
}


static inline void compute_imu_rmat_and_euler_from_quat(void) {
  FLOAT_RMAT_OF_QUAT(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_quat);
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
