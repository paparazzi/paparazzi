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

#include "subsystems/ahrs/ahrs_float_cmpl.h"
#include "subsystems/ahrs/ahrs_float_utils.h"
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


void ahrs_fc_update_mag_full(struct Int32Vect3 *mag, float dt);
void ahrs_fc_update_mag_2d(struct Int32Vect3 *mag, float dt);
void ahrs_fc_update_mag_2d_dumb(struct Int32Vect3 *mag);

struct AhrsFloatCmpl ahrs_fc;

void ahrs_fc_init(void)
{
  ahrs_fc.status = AHRS_FC_UNINIT;
  ahrs_fc.is_aligned = FALSE;

  ahrs_fc.ltp_vel_norm_valid = FALSE;
  ahrs_fc.heading_aligned = FALSE;

  /* init ltp_to_imu quaternion as zero/identity rotation */
  float_quat_identity(&ahrs_fc.ltp_to_imu_quat);

  FLOAT_RATES_ZERO(ahrs_fc.imu_rate);

  /* set default filter cut-off frequency and damping */
  ahrs_fc.accel_omega = AHRS_ACCEL_OMEGA;
  ahrs_fc.accel_zeta = AHRS_ACCEL_ZETA;
  ahrs_fc.mag_omega = AHRS_MAG_OMEGA;
  ahrs_fc.mag_zeta = AHRS_MAG_ZETA;

#if AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  ahrs_fc.correct_gravity = TRUE;
#else
  ahrs_fc.correct_gravity = FALSE;
#endif

  ahrs_fc.gravity_heuristic_factor = AHRS_GRAVITY_HEURISTIC_FACTOR;

  VECT3_ASSIGN(ahrs_fc.mag_h, AHRS_H_X, AHRS_H_Y, AHRS_H_Z);

  ahrs_fc.accel_cnt = 0;
  ahrs_fc.mag_cnt = 0;
}

bool_t ahrs_fc_align(struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                     struct Int32Vect3 *lp_mag)
{

#if USE_MAGNETOMETER
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ahrs_fc.ltp_to_imu_quat, lp_accel, lp_mag);
  ahrs_fc.heading_aligned = TRUE;
#else
  /* Compute an initial orientation from accel and just set heading to zero */
  ahrs_float_get_quat_from_accel(&ahrs_fc.ltp_to_imu_quat, lp_accel);
  ahrs_fc.heading_aligned = FALSE;
#endif

  /* Convert initial orientation from quat to rotation matrix representations. */
  float_rmat_of_quat(&ahrs_fc.ltp_to_imu_rmat, &ahrs_fc.ltp_to_imu_quat);

  /* used averaged gyro as initial value for bias */
  struct Int32Rates bias0;
  RATES_COPY(bias0, *lp_gyro);
  RATES_FLOAT_OF_BFP(ahrs_fc.gyro_bias, bias0);

  ahrs_fc.status = AHRS_FC_RUNNING;
  ahrs_fc.is_aligned = TRUE;

  return TRUE;
}


void ahrs_fc_propagate(struct Int32Rates *gyro, float dt)
{

  /* converts gyro to floating point */
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, *gyro);
  /* unbias measurement */
  RATES_SUB(gyro_float, ahrs_fc.gyro_bias);

#ifdef AHRS_PROPAGATE_LOW_PASS_RATES
  const float alpha = 0.1;
  FLOAT_RATES_LIN_CMB(ahrs_fc.imu_rate, ahrs_fc.imu_rate, (1. - alpha), gyro_float, alpha);
#else
  RATES_COPY(ahrs_fc.imu_rate, gyro_float);
#endif

  /* add correction     */
  struct FloatRates omega;
  RATES_SUM(omega, gyro_float, ahrs_fc.rate_correction);
  /* and zeros it */
  FLOAT_RATES_ZERO(ahrs_fc.rate_correction);

#if AHRS_PROPAGATE_RMAT
  float_rmat_integrate_fi(&ahrs_fc.ltp_to_imu_rmat, &omega, dt);
  float_rmat_reorthogonalize(&ahrs_fc.ltp_to_imu_rmat);
  float_quat_of_rmat(&ahrs_fc.ltp_to_imu_quat, &ahrs_fc.ltp_to_imu_rmat);
#endif
#if AHRS_PROPAGATE_QUAT
  float_quat_integrate(&ahrs_fc.ltp_to_imu_quat, &omega, dt);
  float_quat_normalize(&ahrs_fc.ltp_to_imu_quat);
  float_rmat_of_quat(&ahrs_fc.ltp_to_imu_rmat, &ahrs_fc.ltp_to_imu_quat);
#endif

  // increase accel and mag propagation counters
  ahrs_fc.accel_cnt++;
  ahrs_fc.mag_cnt++;
}

void ahrs_fc_update_accel(struct Int32Vect3 *accel, float dt)
{
  // check if we had at least one propagation since last update
  if (ahrs_fc.accel_cnt == 0) {
    return;
  }

  /* last column of roation matrix = ltp z-axis in imu-frame */
  struct FloatVect3  c2 = { RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 0, 2),
           RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 1, 2),
           RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 2, 2)
  };

  struct FloatVect3 imu_accel_float;
  ACCELS_FLOAT_OF_BFP(imu_accel_float, *accel);

  struct FloatVect3 residual;

  struct FloatVect3 pseudo_gravity_measurement;

  if (ahrs_fc.correct_gravity && ahrs_fc.ltp_vel_norm_valid) {
    /*
     * centrifugal acceleration in body frame
     * a_c_body = omega x (omega x r)
     * (omega x r) = tangential velocity in body frame
     * a_c_body = omega x vel_tangential_body
     * assumption: tangential velocity only along body x-axis
     */
    const struct FloatVect3 vel_tangential_body = {ahrs_fc.ltp_vel_norm, 0.0, 0.0};
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ahrs_fc.body_to_imu);
    struct FloatRates body_rate;
    float_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_fc.imu_rate);
    struct FloatVect3 acc_c_body;
    VECT3_RATES_CROSS_VECT3(acc_c_body, body_rate, vel_tangential_body);

    /* convert centrifugal acceleration from body to imu frame */
    struct FloatVect3 acc_c_imu;
    float_rmat_vmult(&acc_c_imu, body_to_imu_rmat, &acc_c_body);

    /* and subtract it from imu measurement to get a corrected measurement of the gravity vector */
    VECT3_DIFF(pseudo_gravity_measurement, imu_accel_float, acc_c_imu);

  } else {
    VECT3_COPY(pseudo_gravity_measurement, imu_accel_float);
  }

  VECT3_CROSS_PRODUCT(residual, pseudo_gravity_measurement, c2);

  /* FIR filtered pseudo_gravity_measurement */
#define FIR_FILTER_SIZE 8
  static struct FloatVect3 filtered_gravity_measurement = {0., 0., 0.};
  VECT3_SMUL(filtered_gravity_measurement, filtered_gravity_measurement, FIR_FILTER_SIZE - 1);
  VECT3_ADD(filtered_gravity_measurement, pseudo_gravity_measurement);
  VECT3_SDIV(filtered_gravity_measurement, filtered_gravity_measurement, FIR_FILTER_SIZE);

  if (ahrs_fc.gravity_heuristic_factor) {
    /* heuristic on acceleration (gravity estimate) norm */
    /* Factor how strongly to change the weight.
     * e.g. for gravity_heuristic_factor 30:
     * <0.66G = 0, 1G = 1.0, >1.33G = 0
     */

    const float g_meas_norm = float_vect3_norm(&filtered_gravity_measurement) / 9.81;
    ahrs_fc.weight = 1.0 - ahrs_fc.gravity_heuristic_factor * fabs(1.0 - g_meas_norm) / 10.0;
    Bound(ahrs_fc.weight, 0.15, 1.0);
  } else {
    ahrs_fc.weight = 1.0;
  }

  /* Complementary filter proportional gain.
   * Kp = 2 * zeta * omega * weight * ahrs_fc.accel_cnt
   * with ahrs_fc.accel_cnt beeing the number of propagations since last update
   */
  const float gravity_rate_update_gain = -2 * ahrs_fc.accel_zeta * ahrs_fc.accel_omega *
                                         ahrs_fc.weight * ahrs_fc.accel_cnt / 9.81;
  RATES_ADD_SCALED_VECT(ahrs_fc.rate_correction, residual, gravity_rate_update_gain);

  // reset accel propagation counter
  ahrs_fc.accel_cnt = 0;

  /* Complementary filter integral gain
   * Correct the gyro bias.
   * Ki = (omega*weight)^2 * dt
   */
  const float gravity_bias_update_gain = ahrs_fc.accel_omega * ahrs_fc.accel_omega *
                                         ahrs_fc.weight * ahrs_fc.weight * dt / 9.81;
  RATES_ADD_SCALED_VECT(ahrs_fc.gyro_bias, residual, gravity_bias_update_gain);

  /* FIXME: saturate bias */
}


void ahrs_fc_update_mag(struct Int32Vect3 *mag, float dt)
{
  // check if we had at least one propagation since last update
  if (ahrs_fc.mag_cnt == 0) {
    return;
  }
#if AHRS_MAG_UPDATE_ALL_AXES
  ahrs_fc_update_mag_full(mag, dt);
#else
  ahrs_fc_update_mag_2d(mag, dt);
#endif
  // reset mag propagation counter
  ahrs_fc.mag_cnt = 0;
}

void ahrs_fc_update_mag_full(struct Int32Vect3 *mag, float dt)
{

  struct FloatVect3 expected_imu;
  float_rmat_vmult(&expected_imu, &ahrs_fc.ltp_to_imu_rmat, &ahrs_fc.mag_h);

  struct FloatVect3 measured_imu;
  MAGS_FLOAT_OF_BFP(measured_imu, *mag);

  struct FloatVect3 residual_imu;
  VECT3_CROSS_PRODUCT(residual_imu, measured_imu, expected_imu);
  //  DISPLAY_FLOAT_VECT3("# expected", expected_imu);
  //  DISPLAY_FLOAT_VECT3("# measured", measured_imu);
  //  DISPLAY_FLOAT_VECT3("# residual", residual);

  /* Complementary filter proportional gain.
   * Kp = 2 * zeta * omega * weight * ahrs_fc.mag_cnt
   * with ahrs_fc.mag_cnt beeing the number of propagations since last update
   */

  const float mag_rate_update_gain = 2 * ahrs_fc.mag_zeta * ahrs_fc.mag_omega * ahrs_fc.mag_cnt;
  RATES_ADD_SCALED_VECT(ahrs_fc.rate_correction, residual_imu, mag_rate_update_gain);

  /* Complementary filter integral gain
   * Correct the gyro bias.
   * Ki = (omega*weight)^2 * dt
   */
  const float mag_bias_update_gain = -(ahrs_fc.mag_omega * ahrs_fc.mag_omega) * dt;
  RATES_ADD_SCALED_VECT(ahrs_fc.gyro_bias, residual_imu, mag_bias_update_gain);

}

void ahrs_fc_update_mag_2d(struct Int32Vect3 *mag, float dt)
{

  struct FloatVect2 expected_ltp;
  VECT2_COPY(expected_ltp, ahrs_fc.mag_h);
  // normalize expected ltp in 2D (x,y)
  float_vect2_normalize(&expected_ltp);

  struct FloatVect3 measured_imu;
  MAGS_FLOAT_OF_BFP(measured_imu, *mag);
  struct FloatVect3 measured_ltp;
  float_rmat_transp_vmult(&measured_ltp, &ahrs_fc.ltp_to_imu_rmat, &measured_imu);

  struct FloatVect2 measured_ltp_2d = {measured_ltp.x, measured_ltp.y};

  // normalize measured ltp in 2D (x,y)
  float_vect2_normalize(&measured_ltp_2d);

  struct FloatVect3 residual_ltp = {
    0,
    0,
    measured_ltp_2d.x *expected_ltp.y - measured_ltp_2d.y * expected_ltp.x
  };

  //  printf("res : %f\n", residual_ltp.z);

  struct FloatVect3 residual_imu;
  float_rmat_vmult(&residual_imu, &ahrs_fc.ltp_to_imu_rmat, &residual_ltp);


  /* Complementary filter proportional gain.
   * Kp = 2 * zeta * omega * weight * ahrs_fc.mag_cnt
   * with ahrs_fc.mag_cnt beeing the number of propagations since last update
   */
  const float mag_rate_update_gain = 2 * ahrs_fc.mag_zeta * ahrs_fc.mag_omega * ahrs_fc.mag_cnt;
  RATES_ADD_SCALED_VECT(ahrs_fc.rate_correction, residual_imu, mag_rate_update_gain);

  /* Complementary filter integral gain
   * Correct the gyro bias.
   * Ki = (omega*weight)^2 * dt
   */
  const float mag_bias_update_gain = -(ahrs_fc.mag_omega * ahrs_fc.mag_omega) * dt;
  RATES_ADD_SCALED_VECT(ahrs_fc.gyro_bias, residual_imu, mag_bias_update_gain);
}


void ahrs_fc_update_mag_2d_dumb(struct Int32Vect3 *mag)
{

  /* project mag on local tangeant plane */
  struct FloatEulers ltp_to_imu_euler;
  float_eulers_of_rmat(&ltp_to_imu_euler, &ahrs_fc.ltp_to_imu_rmat);
  struct FloatVect3 magf;
  MAGS_FLOAT_OF_BFP(magf, *mag);
  const float cphi   = cosf(ltp_to_imu_euler.phi);
  const float sphi   = sinf(ltp_to_imu_euler.phi);
  const float ctheta = cosf(ltp_to_imu_euler.theta);
  const float stheta = sinf(ltp_to_imu_euler.theta);
  const float mn = ctheta * magf.x + sphi * stheta * magf.y + cphi * stheta * magf.z;
  const float me =     0. * magf.x + cphi       * magf.y - sphi       * magf.z;

  const float res_norm = -RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 0, 0) * me +
                         RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 1, 0) * mn;
  const struct FloatVect3 r2 = {RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 2, 0),
          RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 2, 1),
          RMAT_ELMT(ahrs_fc.ltp_to_imu_rmat, 2, 2)
  };
  const float mag_rate_update_gain = 2.5;
  RATES_ADD_SCALED_VECT(ahrs_fc.rate_correction, r2, (mag_rate_update_gain * res_norm));
  const float mag_bias_update_gain = -2.5e-4;
  RATES_ADD_SCALED_VECT(ahrs_fc.gyro_bias, r2, (mag_bias_update_gain * res_norm));

}

void ahrs_fc_update_gps(struct GpsState *gps_s)
{
#if AHRS_GRAVITY_UPDATE_COORDINATED_TURN && USE_GPS
  if (gps_s->fix == GPS_FIX_3D) {
    ahrs_fc.ltp_vel_norm = gps_s->speed_3d / 100.;
    ahrs_fc.ltp_vel_norm_valid = TRUE;
  } else {
    ahrs_fc.ltp_vel_norm_valid = FALSE;
  }
#endif

#if AHRS_USE_GPS_HEADING && USE_GPS
  //got a 3d fix, ground speed > 0.5 m/s and course accuracy is better than 10deg
  if (3dfix && gps_s->gspeed >= 500 &&
      gps_s->cacc <= RadOfDeg(10 * 1e7)) {

    // gps_s->course is in rad * 1e7, we need it in rad
    float course = gps_s->course / 1e7;

    if (ahrs_fc.heading_aligned) {
      /* the assumption here is that there is no side-slip, so heading=course */
      ahrs_fc_update_heading(course);
    } else {
      /* hard reset the heading if this is the first measurement */
      ahrs_fc_realign_heading(course);
    }
  }
#endif
}


void ahrs_fc_update_heading(float heading)
{

  FLOAT_ANGLE_NORMALIZE(heading);

  struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ahrs_fc.body_to_imu);
  struct FloatQuat ltp_to_body_quat;
  float_quat_comp_inv(&ltp_to_body_quat, &ahrs_fc.ltp_to_imu_quat, body_to_imu_quat);
  struct FloatRMat ltp_to_body_rmat;
  float_rmat_of_quat(&ltp_to_body_rmat, &ltp_to_body_quat);
  // row 0 of ltp_to_body_rmat = body x-axis in ltp frame
  // we only consider x and y
  struct FloatVect2 expected_ltp = {
    RMAT_ELMT(ltp_to_body_rmat, 0, 0),
    RMAT_ELMT(ltp_to_body_rmat, 0, 1)
  };

  // expected_heading cross measured_heading
  struct FloatVect3 residual_ltp = {
    0,
    0,
    expected_ltp.x * sinf(heading) - expected_ltp.y * cosf(heading)
  };

  struct FloatVect3 residual_imu;
  float_rmat_vmult(&residual_imu, &ahrs_fc.ltp_to_imu_rmat, &residual_ltp);

  const float heading_rate_update_gain = 2.5;
  RATES_ADD_SCALED_VECT(ahrs_fc.rate_correction, residual_imu, heading_rate_update_gain);

  float heading_bias_update_gain;
  /* crude attempt to only update bias if deviation is small
   * e.g. needed when you only have gps providing heading
   * and the inital heading is totally different from
   * the gps course information you get once you have a gps fix.
   * Otherwise the bias will be falsely "corrected".
   */
  if (fabs(residual_ltp.z) < sinf(RadOfDeg(5.))) {
    heading_bias_update_gain = -2.5e-4;
  } else {
    heading_bias_update_gain = 0.0;
  }
  RATES_ADD_SCALED_VECT(ahrs_fc.gyro_bias, residual_imu, heading_bias_update_gain);
}


void ahrs_fc_realign_heading(float heading)
{
  FLOAT_ANGLE_NORMALIZE(heading);

  /* quaternion representing only the heading rotation from ltp to body */
  struct FloatQuat q_h_new;
  q_h_new.qx = 0.0;
  q_h_new.qy = 0.0;
  q_h_new.qz = sinf(heading / 2.0);
  q_h_new.qi = cosf(heading / 2.0);

  struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ahrs_fc.body_to_imu);
  struct FloatQuat ltp_to_body_quat;
  float_quat_comp_inv(&ltp_to_body_quat, &ahrs_fc.ltp_to_imu_quat, body_to_imu_quat);
  /* quaternion representing current heading only */
  struct FloatQuat q_h;
  QUAT_COPY(q_h, ltp_to_body_quat);
  q_h.qx = 0.;
  q_h.qy = 0.;
  float_quat_normalize(&q_h);

  /* quaternion representing rotation from current to new heading */
  struct FloatQuat q_c;
  float_quat_inv_comp_norm_shortest(&q_c, &q_h, &q_h_new);

  /* correct current heading in body frame */
  struct FloatQuat q;
  float_quat_comp_norm_shortest(&q, &q_c, &ltp_to_body_quat);

  /* compute ltp to imu rotation quaternion and matrix */
  float_quat_comp(&ahrs_fc.ltp_to_imu_quat, &q, body_to_imu_quat);
  float_rmat_of_quat(&ahrs_fc.ltp_to_imu_rmat, &ahrs_fc.ltp_to_imu_quat);

  ahrs_fc.heading_aligned = TRUE;
}

void ahrs_fc_set_body_to_imu(struct OrientationReps *body_to_imu)
{
  ahrs_fc_set_body_to_imu_quat(orientationGetQuat_f(body_to_imu));
}

void ahrs_fc_set_body_to_imu_quat(struct FloatQuat *q_b2i)
{
  orientationSetQuat_f(&ahrs_fc.body_to_imu, q_b2i);

  if (!ahrs_fc.is_aligned) {
    /* Set ltp_to_imu so that body is zero */
    memcpy(&ahrs_fc.ltp_to_imu_quat, orientationGetQuat_f(&ahrs_fc.body_to_imu),
           sizeof(struct FloatQuat));
    memcpy(&ahrs_fc.ltp_to_imu_rmat, orientationGetRMat_f(&ahrs_fc.body_to_imu),
           sizeof(struct FloatRMat));
  }
}
