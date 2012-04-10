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

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_float_cmpl.h"
#include "subsystems/ahrs/ahrs_float_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/imu.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_simple_matrix.h"
#include "generated/airframe.h"
#if USE_GPS
#include "subsystems/gps.h"
#endif


//#include "../../test/pprz_algebra_print.h"

#if AHRS_PROPAGATE_RMAT && AHRS_PROPAGATE_QUAT
#error "You can only define either AHRS_PROPAGATE_RMAT or AHRS_PROPAGATE_QUAT, not both!"
#endif
#if !AHRS_PROPAGATE_RMAT && !AHRS_PROPAGATE_QUAT
#error "You have to define either AHRS_PROPAGATE_RMAT or AHRS_PROPAGATE_QUAT"
#endif

#ifdef AHRS_MAG_UPDATE_YAW_ONLY
#warning "AHRS_MAG_UPDATE_YAW_ONLY is deprecated, please remove it. This is the default behaviour. Define AHRS_MAG_UPDATE_ALL_AXES to use mag for all axes and not only yaw."
#endif

#if USE_MAGNETOMETER && AHRS_USE_GPS_HEADING
#warning "Using magnetometer and GPS course to update heading. Probably better to set USE_MAGNETOMETER=0 if you want to use GPS course."
#endif

void ahrs_update_mag_full(void);
void ahrs_update_mag_2d(void);
void ahrs_update_mag_2d_dumb(void);

static inline void compute_imu_quat_and_euler_from_rmat(void);
static inline void compute_imu_rmat_and_euler_from_quat(void);
static inline void compute_body_orientation_and_rates(void);


struct AhrsFloatCmplRmat ahrs_impl;

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;
  ahrs_impl.ltp_vel_norm_valid = FALSE;
  ahrs_impl.heading_aligned = FALSE;

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

#if AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  ahrs_impl.correct_gravity = TRUE;
#else
  ahrs_impl.correct_gravity = FALSE;
#endif

}

void ahrs_align(void) {

#if USE_MAGNETOMETER
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ahrs_float.ltp_to_imu_quat, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);
  ahrs_impl.heading_aligned = TRUE;
#else
  /* Compute an initial orientation from accel and just set heading to zero */
  ahrs_float_get_quat_from_accel(&ahrs_float.ltp_to_imu_quat, &ahrs_aligner.lp_accel);
  ahrs_impl.heading_aligned = FALSE;
#endif

  /* Convert initial orientation from quat to euler and rotation matrix representations. */
  compute_imu_rmat_and_euler_from_quat();

  /* Compute initial body orientation */
  compute_body_orientation_and_rates();

  /* compute fixed point representations */
  AHRS_INT_OF_FLOAT();
  AHRS_IMU_INT_OF_FLOAT();

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
#if AHRS_PROPAGATE_RMAT
  FLOAT_RMAT_INTEGRATE_FI(ahrs_float.ltp_to_imu_rmat, omega, dt );
  float_rmat_reorthogonalize(&ahrs_float.ltp_to_imu_rmat);
  compute_imu_quat_and_euler_from_rmat();
#endif
#if AHRS_PROPAGATE_QUAT
  FLOAT_QUAT_INTEGRATE(ahrs_float.ltp_to_imu_quat, omega, dt);
  FLOAT_QUAT_NORMALIZE(ahrs_float.ltp_to_imu_quat);
  compute_imu_rmat_and_euler_from_quat();
#endif
  compute_body_orientation_and_rates();

  /* compute fixed point representations */
  AHRS_INT_OF_FLOAT();
  AHRS_IMU_INT_OF_FLOAT();
}

void ahrs_update_accel(void) {

  /* last column of roation matrix = ltp z-axis in imu-frame */
  struct FloatVect3  c2 = { RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 0,2),
                            RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 1,2),
                            RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,2)};

  struct FloatVect3 imu_accel_float;
  ACCELS_FLOAT_OF_BFP(imu_accel_float, imu.accel);

  struct FloatVect3 residual;

  if (ahrs_impl.correct_gravity && ahrs_impl.ltp_vel_norm_valid) {
    /*
     * centrifugal acceleration in body frame
     * a_c_body = omega x (omega x r)
     * (omega x r) = tangential velocity in body frame
     * a_c_body = omega x vel_tangential_body
     * assumption: tangential velocity only along body x-axis
     */
    const struct FloatVect3 vel_tangential_body = {ahrs_impl.ltp_vel_norm, 0.0, 0.0};
    struct FloatVect3 acc_c_body;
    VECT3_RATES_CROSS_VECT3(acc_c_body, ahrs_float.body_rate, vel_tangential_body);

    /* convert centrifucal acceleration from body to imu frame */
    struct FloatVect3 acc_c_imu;
    FLOAT_RMAT_VECT3_MUL(acc_c_imu, ahrs_impl.body_to_imu_rmat, acc_c_body);

    /* and subtract it from imu measurement to get a corrected measurement of the gravitiy vector */
    struct FloatVect3 corrected_gravity;
    VECT3_DIFF(corrected_gravity, imu_accel_float, acc_c_imu);

    /* compute the residual of gravity vector in imu frame */
    FLOAT_VECT3_CROSS_PRODUCT(residual, corrected_gravity, c2);
  } else {
    FLOAT_VECT3_CROSS_PRODUCT(residual, imu_accel_float, c2);
  }

#ifdef AHRS_GRAVITY_UPDATE_NORM_HEURISTIC
  /* heuristic on acceleration norm */
  const float acc_norm = FLOAT_VECT3_NORM(imu_accel_float);
  const float weight = Chop(1.-6*fabs((9.81-acc_norm)/9.81), 0., 1.);
#else
  const float weight = 1.;
#endif

  /* compute correction */
  const float gravity_rate_update_gain = -5e-2; // -5e-2
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual, weight*gravity_rate_update_gain);

  const float gravity_bias_update_gain = 1e-5; // -5e-6
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual, weight*gravity_bias_update_gain);

  /* FIXME: saturate bias */

}


void ahrs_update_mag(void) {
#if AHRS_MAG_UPDATE_ALL_AXES
  ahrs_update_mag_full();
#else
  ahrs_update_mag_2d();
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


void ahrs_update_mag_2d_dumb(void) {

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
  const struct FloatVect3 r2 = {RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,0),
                                RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,1),
                                RMAT_ELMT(ahrs_float.ltp_to_imu_rmat, 2,2)};
  const float mag_rate_update_gain = 2.5;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, r2, (mag_rate_update_gain*res_norm));
  const float mag_bias_update_gain = -2.5e-4;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, r2, (mag_bias_update_gain*res_norm));

}

void ahrs_update_gps(void) {
#if AHRS_GRAVITY_UPDATE_COORDINATED_TURN && USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    ahrs_impl.ltp_vel_norm = gps.speed_3d / 100.;
    ahrs_impl.ltp_vel_norm_valid = TRUE;
  } else {
    ahrs_impl.ltp_vel_norm_valid = FALSE;
  }
#endif

#if AHRS_USE_GPS_HEADING && USE_GPS
  //got a 3d fix, ground speed > 0.5 m/s and course accuracy is better than 10deg
  if(gps.fix == GPS_FIX_3D &&
     gps.gspeed >= 500 &&
     gps.cacc <= RadOfDeg(10*1e7)) {

    // gps.course is in rad * 1e7, we need it in rad
    float course = gps.course / 1e7;

    if (ahrs_impl.heading_aligned) {
      /* the assumption here is that there is no side-slip, so heading=course */
      ahrs_update_heading(course);
    }
    else {
      /* hard reset the heading if this is the first measurement */
      ahrs_realign_heading(course);
    }
  }
#endif
}


void ahrs_update_heading(float heading) {

  FLOAT_ANGLE_NORMALIZE(heading);

  // row 0 of ltp_to_body_rmat = body x-axis in ltp frame
  // we only consider x and y
  struct FloatVect2 expected_ltp =
    { RMAT_ELMT(ahrs_float.ltp_to_body_rmat, 0, 0),
      RMAT_ELMT(ahrs_float.ltp_to_body_rmat, 0, 1) };

  // expected_heading cross measured_heading
  struct FloatVect3 residual_ltp =
    { 0,
      0,
      expected_ltp.x * sinf(heading) - expected_ltp.y * cosf(heading)};

  struct FloatVect3 residual_imu;
  FLOAT_RMAT_VECT3_MUL(residual_imu, ahrs_float.ltp_to_imu_rmat, residual_ltp);

  const float heading_rate_update_gain = 2.5;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual_imu, heading_rate_update_gain);

  float heading_bias_update_gain;
  /* crude attempt to only update bias if deviation is small
   * e.g. needed when you only have gps providing heading
   * and the inital heading is totally different from
   * the gps course information you get once you have a gps fix.
   * Otherwise the bias will be falsely "corrected".
   */
  if (fabs(residual_ltp.z) < sinf(RadOfDeg(5.)))
    heading_bias_update_gain = -2.5e-4;
  else
    heading_bias_update_gain = 0.0;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual_imu, heading_bias_update_gain);
}


void ahrs_realign_heading(float heading) {
  FLOAT_ANGLE_NORMALIZE(heading);

  /* quaternion representing only the heading rotation from ltp to body */
  struct FloatQuat q_h_new;
  q_h_new.qx = 0.0;
  q_h_new.qy = 0.0;
  q_h_new.qz = sinf(heading/2.0);
  q_h_new.qi = cosf(heading/2.0);

  /* quaternion representing current heading only */
  struct FloatQuat q_h;
  QUAT_COPY(q_h, ahrs_float.ltp_to_body_quat);
  q_h.qx = 0.;
  q_h.qy = 0.;
  FLOAT_QUAT_NORMALIZE(q_h);

  /* quaternion representing rotation from current to new heading */
  struct FloatQuat q_c;
  FLOAT_QUAT_INV_COMP_NORM_SHORTEST(q_c, q_h, q_h_new);

  /* correct current heading in body frame */
  struct FloatQuat q;
  FLOAT_QUAT_COMP_NORM_SHORTEST(q, q_c, ahrs_float.ltp_to_body_quat);
  QUAT_COPY(ahrs_float.ltp_to_body_quat, q);

  /* compute ltp to imu rotation quaternion */
  FLOAT_QUAT_COMP(ahrs_float.ltp_to_imu_quat,
                  ahrs_float.ltp_to_body_quat, ahrs_impl.body_to_imu_quat);

  /* compute other body orientation representations */
  FLOAT_RMAT_OF_QUAT(ahrs_float.ltp_to_body_rmat, ahrs_float.ltp_to_body_quat);
  FLOAT_EULERS_OF_RMAT(ahrs_float.ltp_to_body_euler, ahrs_float.ltp_to_body_rmat);

  /* compute other ltp to imu rotation representations */
  compute_imu_rmat_and_euler_from_quat();

  /* compute fixed point representations */
  AHRS_INT_OF_FLOAT();
  AHRS_IMU_INT_OF_FLOAT();

  ahrs_impl.heading_aligned = TRUE;
}


/**
 * Compute ltp to imu rotation in euler angles and quaternion representations
 * from the rotation matrix representation
 */
static inline void compute_imu_quat_and_euler_from_rmat(void) {
  FLOAT_QUAT_OF_RMAT(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_rmat);
  FLOAT_EULERS_OF_RMAT(ahrs_float.ltp_to_imu_euler, ahrs_float.ltp_to_imu_rmat);
}


static inline void compute_imu_rmat_and_euler_from_quat(void) {
  FLOAT_RMAT_OF_QUAT(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_quat);
  FLOAT_EULERS_OF_RMAT(ahrs_float.ltp_to_imu_euler, ahrs_float.ltp_to_imu_rmat);
}



/**
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


#ifdef AHRS_UPDATE_FW_ESTIMATOR
// TODO use ahrs result directly
#include "estimator.h"
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
void ahrs_update_fw_estimator(void)
{
  // export results to estimator
  estimator_phi   = ahrs_float.ltp_to_body_euler.phi - ins_roll_neutral;
  estimator_theta = ahrs_float.ltp_to_body_euler.theta - ins_pitch_neutral;
  estimator_psi   = ahrs_float.ltp_to_body_euler.psi;

  estimator_p = ahrs_float.body_rate.p;
  estimator_q = ahrs_float.body_rate.q;
  estimator_r = ahrs_float.body_rate.r;
}
#endif //AHRS_UPDATE_FW_ESTIMATOR
