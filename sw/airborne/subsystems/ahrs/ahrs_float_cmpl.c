/*
 * Copyright (C) 2010-2013 The Paparazzi Team
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
 * @file subsystems/ahrs/ahrs_float_cmpl.c
 *
 * Complementary filter in float to estimate the attitude, heading and gyro bias.
 *
 * Propagation can be done in rotation matrix or quaternion representation.
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
#warning "Using magnetometer _and_ GPS course to update heading. Probably better to <configure name="USE_MAGNETOMETER" value="0"/> if you want to use GPS course."
#endif

#ifndef AHRS_PROPAGATE_FREQUENCY
#define AHRS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif
PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)

#ifndef AHRS_CORRECT_FREQUENCY
#define AHRS_CORRECT_FREQUENCY AHRS_PROPAGATE_FREQUENCY
#endif
PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)

#ifndef AHRS_MAG_CORRECT_FREQUENCY
#define AHRS_MAG_CORRECT_FREQUENCY 50
#endif
PRINT_CONFIG_VAR(AHRS_MAG_CORRECT_FREQUENCY)

/*
 * default gains for correcting attitude and bias from accel/mag
 */
#ifndef AHRS_ACCEL_OMEGA
#define AHRS_ACCEL_OMEGA 0.063
#endif
#ifndef AHRS_ACCEL_ZETA
#define AHRS_ACCEL_ZETA 0.9
#endif

#ifndef AHRS_MAG_OMEGA
#define AHRS_MAG_OMEGA 0.04
#endif
#ifndef AHRS_MAG_ZETA
#define AHRS_MAG_ZETA 0.9
#endif

/** by default use the gravity heuristic to reduce gain */
#ifndef AHRS_GRAVITY_HEURISTIC_FACTOR
#define AHRS_GRAVITY_HEURISTIC_FACTOR 30
#endif

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
#endif //AHRS_UPDATE_FW_ESTIMATOR


void ahrs_update_mag_full(void);
void ahrs_update_mag_2d(void);
void ahrs_update_mag_2d_dumb(void);

static inline void compute_body_orientation_and_rates(void);


struct AhrsFloatCmpl ahrs_impl;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att(void) {
  struct FloatEulers ltp_to_imu_euler;
  FLOAT_EULERS_OF_QUAT(ltp_to_imu_euler, ahrs_impl.ltp_to_imu_quat);
  struct Int32Eulers euler_i;
  EULERS_BFP_OF_REAL(euler_i, ltp_to_imu_euler);
  struct Int32Eulers* eulers_body = stateGetNedToBodyEulers_i();
  DOWNLINK_SEND_AHRS_EULER_INT(DefaultChannel, DefaultDevice,
      &euler_i.phi,
      &euler_i.theta,
      &euler_i.psi,
      &(eulers_body->phi),
      &(eulers_body->theta),
      &(eulers_body->psi));
}

static void send_geo_mag(void) {
  DOWNLINK_SEND_GEO_MAG(DefaultChannel, DefaultDevice,
                        &ahrs_impl.mag_h.x, &ahrs_impl.mag_h.y, &ahrs_impl.mag_h.z);
}

// TODO convert from float to int if we really need this one
/*
static void send_rmat(void) {
  struct Int32RMat* att_rmat = stateGetNedToBodyRMat_i();
  DOWNLINK_SEND_AHRS_RMAT(DefaultChannel, DefaultDevice,
      &ahrs_impl.ltp_to_imu_rmat.m[0],
      &ahrs_impl.ltp_to_imu_rmat.m[1],
      &ahrs_impl.ltp_to_imu_rmat.m[2],
      &ahrs_impl.ltp_to_imu_rmat.m[3],
      &ahrs_impl.ltp_to_imu_rmat.m[4],
      &ahrs_impl.ltp_to_imu_rmat.m[5],
      &ahrs_impl.ltp_to_imu_rmat.m[6],
      &ahrs_impl.ltp_to_imu_rmat.m[7],
      &ahrs_impl.ltp_to_imu_rmat.m[8],
      &(att_rmat->m[0]),
      &(att_rmat->m[1]),
      &(att_rmat->m[2]),
      &(att_rmat->m[3]),
      &(att_rmat->m[4]),
      &(att_rmat->m[5]),
      &(att_rmat->m[6]),
      &(att_rmat->m[7]),
      &(att_rmat->m[8]));
}
*/
#endif

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;
  ahrs_impl.ltp_vel_norm_valid = FALSE;
  ahrs_impl.heading_aligned = FALSE;

  /* Initialises IMU alignement */
  struct FloatEulers body_to_imu_euler =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  FLOAT_QUAT_OF_EULERS(ahrs_impl.body_to_imu_quat, body_to_imu_euler);
  FLOAT_RMAT_OF_EULERS(ahrs_impl.body_to_imu_rmat, body_to_imu_euler);

  /* Set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_impl.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  RMAT_COPY(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.body_to_imu_rmat);

  FLOAT_RATES_ZERO(ahrs_impl.imu_rate);

  /* set default filter cut-off frequency and damping */
  ahrs_impl.accel_omega = AHRS_ACCEL_OMEGA;
  ahrs_impl.accel_zeta = AHRS_ACCEL_ZETA;
  ahrs_impl.mag_omega = AHRS_MAG_OMEGA;
  ahrs_impl.mag_zeta = AHRS_MAG_ZETA;

#if AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  ahrs_impl.correct_gravity = TRUE;
#else
  ahrs_impl.correct_gravity = FALSE;
#endif

  ahrs_impl.gravity_heuristic_factor = AHRS_GRAVITY_HEURISTIC_FACTOR;

  VECT3_ASSIGN(ahrs_impl.mag_h, AHRS_H_X, AHRS_H_Y, AHRS_H_Z);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "AHRS_EULER_INT", send_att);
  register_periodic_telemetry(DefaultPeriodic, "GEO_MAG", send_geo_mag);
#endif
}

void ahrs_align(void) {

#if USE_MAGNETOMETER
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ahrs_impl.ltp_to_imu_quat, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);
  ahrs_impl.heading_aligned = TRUE;
#else
  /* Compute an initial orientation from accel and just set heading to zero */
  ahrs_float_get_quat_from_accel(&ahrs_impl.ltp_to_imu_quat, &ahrs_aligner.lp_accel);
  ahrs_impl.heading_aligned = FALSE;
#endif

  /* Convert initial orientation from quat to rotation matrix representations. */
  FLOAT_RMAT_OF_QUAT(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);

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
  FLOAT_RATES_LIN_CMB(ahrs_impl.imu_rate, ahrs_impl.imu_rate, (1.-alpha), gyro_float, alpha);
#else
  RATES_COPY(ahrs_impl.imu_rate,gyro_float);
#endif

  /* add correction     */
  struct FloatRates omega;
  RATES_SUM(omega, gyro_float, ahrs_impl.rate_correction);
  /* and zeros it */
  FLOAT_RATES_ZERO(ahrs_impl.rate_correction);

  const float dt = 1./AHRS_PROPAGATE_FREQUENCY;
#if AHRS_PROPAGATE_RMAT
  FLOAT_RMAT_INTEGRATE_FI(ahrs_impl.ltp_to_imu_rmat, omega, dt);
  float_rmat_reorthogonalize(&ahrs_impl.ltp_to_imu_rmat);
  FLOAT_QUAT_OF_RMAT(ahrs_impl.ltp_to_imu_quat, ahrs_impl.ltp_to_imu_rmat);
#endif
#if AHRS_PROPAGATE_QUAT
  FLOAT_QUAT_INTEGRATE(ahrs_impl.ltp_to_imu_quat, omega, dt);
  FLOAT_QUAT_NORMALIZE(ahrs_impl.ltp_to_imu_quat);
  FLOAT_RMAT_OF_QUAT(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);
#endif
  compute_body_orientation_and_rates();

}

void ahrs_update_accel(void) {

  /* last column of roation matrix = ltp z-axis in imu-frame */
  struct FloatVect3  c2 = { RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 0,2),
                            RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 1,2),
                            RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 2,2)};

  struct FloatVect3 imu_accel_float;
  ACCELS_FLOAT_OF_BFP(imu_accel_float, imu.accel);

  struct FloatVect3 residual;

  struct FloatVect3 pseudo_gravity_measurement;

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
    VECT3_RATES_CROSS_VECT3(acc_c_body, *stateGetBodyRates_f(), vel_tangential_body);

    /* convert centrifugal acceleration from body to imu frame */
    struct FloatVect3 acc_c_imu;
    FLOAT_RMAT_VECT3_MUL(acc_c_imu, ahrs_impl.body_to_imu_rmat, acc_c_body);

    /* and subtract it from imu measurement to get a corrected measurement of the gravity vector */
    VECT3_DIFF(pseudo_gravity_measurement, imu_accel_float, acc_c_imu);

  } else {
    VECT3_COPY(pseudo_gravity_measurement, imu_accel_float);
  }

  FLOAT_VECT3_CROSS_PRODUCT(residual, pseudo_gravity_measurement, c2);

  /* FIR filtered pseudo_gravity_measurement */
  #define FIR_FILTER_SIZE 8
  static struct FloatVect3 filtered_gravity_measurement = {0., 0., 0.};
  VECT3_SMUL(filtered_gravity_measurement, filtered_gravity_measurement, FIR_FILTER_SIZE-1);
  VECT3_ADD(filtered_gravity_measurement, pseudo_gravity_measurement);
  VECT3_SDIV(filtered_gravity_measurement, filtered_gravity_measurement, FIR_FILTER_SIZE);

  if (ahrs_impl.gravity_heuristic_factor) {
    /* heuristic on acceleration (gravity estimate) norm */
    /* Factor how strongly to change the weight.
     * e.g. for gravity_heuristic_factor 30:
     * <0.66G = 0, 1G = 1.0, >1.33G = 0
     */

    const float g_meas_norm = FLOAT_VECT3_NORM(filtered_gravity_measurement) / 9.81;
    ahrs_impl.weight = 1.0 - ahrs_impl.gravity_heuristic_factor * fabs(1.0 - g_meas_norm) / 10.0;
    Bound(ahrs_impl.weight, 0.15, 1.0);
  }
  else {
    ahrs_impl.weight = 1.0;
  }

  /* Complementary filter proportional gain.
   * Kp = 2 * zeta * omega * weight * AHRS_PROPAGATE_FREQUENCY / AHRS_CORRECT_FREQUENCY
   */
  const float gravity_rate_update_gain = -2 * ahrs_impl.accel_zeta * ahrs_impl.accel_omega *
    ahrs_impl.weight * AHRS_PROPAGATE_FREQUENCY / (AHRS_CORRECT_FREQUENCY * 9.81);
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual, gravity_rate_update_gain);

  /* Complementary filter integral gain
   * Correct the gyro bias.
   * Ki = (omega*weight)^2/AHRS_CORRECT_FREQUENCY
   */
  const float gravity_bias_update_gain = ahrs_impl.accel_omega * ahrs_impl.accel_omega *
    ahrs_impl.weight * ahrs_impl.weight / (AHRS_CORRECT_FREQUENCY * 9.81);
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual, gravity_bias_update_gain);

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

  struct FloatVect3 expected_imu;
  FLOAT_RMAT_VECT3_MUL(expected_imu, ahrs_impl.ltp_to_imu_rmat, ahrs_impl.mag_h);

  struct FloatVect3 measured_imu;
  MAGS_FLOAT_OF_BFP(measured_imu, imu.mag);

  struct FloatVect3 residual_imu;
  FLOAT_VECT3_CROSS_PRODUCT(residual_imu, measured_imu, expected_imu);
  //  DISPLAY_FLOAT_VECT3("# expected", expected_imu);
  //  DISPLAY_FLOAT_VECT3("# measured", measured_imu);
  //  DISPLAY_FLOAT_VECT3("# residual", residual);

  /* Complementary filter proportional gain.
   * Kp = 2 * zeta * omega * weight * AHRS_PROPAGATE_FREQUENCY / AHRS_MAG_CORRECT_FREQUENCY
   */

  const float mag_rate_update_gain = 2 * ahrs_impl.mag_zeta * ahrs_impl.mag_omega *
    AHRS_PROPAGATE_FREQUENCY / AHRS_MAG_CORRECT_FREQUENCY;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual_imu, mag_rate_update_gain);

  /* Complementary filter integral gain
   * Correct the gyro bias.
   * Ki = (omega*weight)^2/AHRS_CORRECT_FREQUENCY
   */
  const float mag_bias_update_gain = -(ahrs_impl.mag_omega * ahrs_impl.mag_omega) /
    AHRS_MAG_CORRECT_FREQUENCY;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual_imu, mag_bias_update_gain);

}

void ahrs_update_mag_2d(void) {

  struct FloatVect2 expected_ltp;
  VECT2_COPY(expected_ltp, ahrs_impl.mag_h);
  // normalize expected ltp in 2D (x,y)
  FLOAT_VECT2_NORMALIZE(expected_ltp);

  struct FloatVect3 measured_imu;
  MAGS_FLOAT_OF_BFP(measured_imu, imu.mag);
  struct FloatVect3 measured_ltp;
  FLOAT_RMAT_VECT3_TRANSP_MUL(measured_ltp, ahrs_impl.ltp_to_imu_rmat, measured_imu);

  struct FloatVect2 measured_ltp_2d={measured_ltp.x, measured_ltp.y};

  // normalize measured ltp in 2D (x,y)
  FLOAT_VECT2_NORMALIZE(measured_ltp_2d);

  const struct FloatVect3 residual_ltp =
    { 0,
      0,
      measured_ltp_2d.x * expected_ltp.y - measured_ltp_2d.y * expected_ltp.x };

  //  printf("res : %f\n", residual_ltp.z);

  struct FloatVect3 residual_imu;
  FLOAT_RMAT_VECT3_MUL(residual_imu, ahrs_impl.ltp_to_imu_rmat, residual_ltp);


  /* Complementary filter proportional gain.
   * Kp = 2 * zeta * omega * weight * AHRS_PROPAGATE_FREQUENCY / AHRS_MAG_CORRECT_FREQUENCY
   */
  const float mag_rate_update_gain = 2 * ahrs_impl.mag_zeta * ahrs_impl.mag_omega *
    AHRS_PROPAGATE_FREQUENCY / AHRS_MAG_CORRECT_FREQUENCY;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.rate_correction, residual_imu, mag_rate_update_gain);

  /* Complementary filter integral gain
   * Correct the gyro bias.
   * Ki = (omega*weight)^2/AHRS_CORRECT_FREQUENCY
   */
  const float mag_bias_update_gain = -(ahrs_impl.mag_omega * ahrs_impl.mag_omega) /
    AHRS_MAG_CORRECT_FREQUENCY;
  FLOAT_RATES_ADD_SCALED_VECT(ahrs_impl.gyro_bias, residual_imu, mag_bias_update_gain);
}


void ahrs_update_mag_2d_dumb(void) {

  /* project mag on local tangeant plane */
  struct FloatEulers ltp_to_imu_euler;
  FLOAT_EULERS_OF_RMAT(ltp_to_imu_euler, ahrs_impl.ltp_to_imu_rmat);
  struct FloatVect3 magf;
  MAGS_FLOAT_OF_BFP(magf, imu.mag);
  const float cphi   = cosf(ltp_to_imu_euler.phi);
  const float sphi   = sinf(ltp_to_imu_euler.phi);
  const float ctheta = cosf(ltp_to_imu_euler.theta);
  const float stheta = sinf(ltp_to_imu_euler.theta);
  const float mn = ctheta * magf.x + sphi*stheta*magf.y + cphi*stheta*magf.z;
  const float me =     0. * magf.x + cphi       *magf.y - sphi       *magf.z;

  const float res_norm = -RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 0,0) * me +
                          RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 1,0) * mn;
  const struct FloatVect3 r2 = {RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 2,0),
                                RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 2,1),
                                RMAT_ELMT(ahrs_impl.ltp_to_imu_rmat, 2,2)};
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

  struct FloatRMat* ltp_to_body_rmat = stateGetNedToBodyRMat_f();
  // row 0 of ltp_to_body_rmat = body x-axis in ltp frame
  // we only consider x and y
  struct FloatVect2 expected_ltp =
    { RMAT_ELMT(*ltp_to_body_rmat, 0, 0),
      RMAT_ELMT(*ltp_to_body_rmat, 0, 1) };

  // expected_heading cross measured_heading
  struct FloatVect3 residual_ltp =
    { 0,
      0,
      expected_ltp.x * sinf(heading) - expected_ltp.y * cosf(heading)};

  struct FloatVect3 residual_imu;
  FLOAT_RMAT_VECT3_MUL(residual_imu, ahrs_impl.ltp_to_imu_rmat, residual_ltp);

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

  struct FloatQuat* ltp_to_body_quat = stateGetNedToBodyQuat_f();
  /* quaternion representing current heading only */
  struct FloatQuat q_h;
  QUAT_COPY(q_h, *ltp_to_body_quat);
  q_h.qx = 0.;
  q_h.qy = 0.;
  FLOAT_QUAT_NORMALIZE(q_h);

  /* quaternion representing rotation from current to new heading */
  struct FloatQuat q_c;
  FLOAT_QUAT_INV_COMP_NORM_SHORTEST(q_c, q_h, q_h_new);

  /* correct current heading in body frame */
  struct FloatQuat q;
  FLOAT_QUAT_COMP_NORM_SHORTEST(q, q_c, *ltp_to_body_quat);

  /* compute ltp to imu rotation quaternion and matrix */
  FLOAT_QUAT_COMP(ahrs_impl.ltp_to_imu_quat, q, ahrs_impl.body_to_imu_quat);
  FLOAT_RMAT_OF_QUAT(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);

  /* set state */
  stateSetNedToBodyQuat_f(&q);

  ahrs_impl.heading_aligned = TRUE;
}


/**
 * Compute body orientation and rates from imu orientation and rates
 */
static inline void compute_body_orientation_and_rates(void) {
  /* Compute LTP to BODY quaternion */
  struct FloatQuat ltp_to_body_quat;
  FLOAT_QUAT_COMP_INV(ltp_to_body_quat, ahrs_impl.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  /* Set state */
#ifdef AHRS_UPDATE_FW_ESTIMATOR
  struct FloatEulers neutrals_to_body_eulers = { ins_roll_neutral, ins_pitch_neutral, 0. };
  struct FloatQuat neutrals_to_body_quat, ltp_to_neutrals_quat;
  FLOAT_QUAT_OF_EULERS(neutrals_to_body_quat, neutrals_to_body_eulers);
  FLOAT_QUAT_NORMALIZE(neutrals_to_body_quat);
  FLOAT_QUAT_COMP_INV(ltp_to_neutrals_quat, ltp_to_body_quat, neutrals_to_body_quat);
  stateSetNedToBodyQuat_f(&ltp_to_neutrals_quat);
#else
  stateSetNedToBodyQuat_f(&ltp_to_body_quat);
#endif

  /* compute body rates */
  struct FloatRates body_rate;
  FLOAT_RMAT_TRANSP_RATEMULT(body_rate, ahrs_impl.body_to_imu_rmat, ahrs_impl.imu_rate);
  stateSetBodyRates_f(&body_rate);
}
