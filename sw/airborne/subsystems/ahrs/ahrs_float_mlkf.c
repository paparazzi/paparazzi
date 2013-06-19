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
 * @file subsystems/ahrs/ahrs_float_mlkf.c
 *
 * Multiplicative linearized Kalman Filter in quaternion formulation.
 *
 * Estimate the attitude, heading and gyro bias.
 */

#include "subsystems/ahrs/ahrs_float_mlkf.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/ahrs/ahrs_float_utils.h"

#include <float.h>   /* for FLT_MIN     */
#include <string.h>  /* for memcpy      */
#include <math.h>    /* for M_PI        */

#include "state.h"

#include "subsystems/imu.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_simple_matrix.h"
#include "generated/airframe.h"

//#include <stdio.h>

#ifndef AHRS_PROPAGATE_FREQUENCY
#define AHRS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif

static inline void propagate_ref(void);
static inline void propagate_state(void);
static inline void update_state(const struct FloatVect3 *i_expected, struct FloatVect3* b_measured, const float noise[]);
static inline void reset_state(void);
static inline void set_body_state_from_quat(void);

struct AhrsMlkf ahrs_impl;


void ahrs_init(void) {

  ahrs.status = AHRS_UNINIT;

  /*
   * Initialises our IMU alignement variables
   * This should probably done in the IMU code instead
   */
  struct FloatEulers body_to_imu_euler =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  FLOAT_QUAT_OF_EULERS(ahrs_impl.body_to_imu_quat, body_to_imu_euler);
  FLOAT_RMAT_OF_EULERS(ahrs_impl.body_to_imu_rmat, body_to_imu_euler);

  /* Set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_impl.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  RMAT_COPY(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.body_to_imu_rmat);

  FLOAT_RATES_ZERO(ahrs_impl.imu_rate);

  /*
   * Initialises our state
   */
  FLOAT_RATES_ZERO(ahrs_impl.gyro_bias);
  const float P0_a = 1.;
  const float P0_b = 1e-4;
  float P0[6][6] = {{ P0_a, 0.,   0.,   0.,   0.,   0.  },
                    { 0.,   P0_a, 0.,   0.,   0.,   0.  },
                    { 0.,   0.,   P0_a, 0.,   0.,   0.  },
                    { 0.,   0.,   0.,   P0_b, 0.,   0.  },
                    { 0.,   0.,   0.,   0.,   P0_b, 0.  },
                    { 0.,   0.,   0.,   0.,   0.,   P0_b}};
  memcpy(ahrs_impl.P, P0, sizeof(P0));

}

void ahrs_align(void) {

  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_float_get_quat_from_accel_mag(&ahrs_impl.ltp_to_imu_quat, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);

  /* Convert initial orientation from quat to rotation matrix representations. */
  FLOAT_RMAT_OF_QUAT(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);

  /* set initial body orientation */
  set_body_state_from_quat();

  /* used averaged gyro as initial value for bias */
  struct Int32Rates bias0;
  RATES_COPY(bias0, ahrs_aligner.lp_gyro);
  RATES_FLOAT_OF_BFP(ahrs_impl.gyro_bias, bias0);

  ahrs.status = AHRS_RUNNING;
}

void ahrs_propagate(void) {
  propagate_ref();
  propagate_state();
  set_body_state_from_quat();
}

void ahrs_update_accel(void) {
  struct FloatVect3 imu_g;
  ACCELS_FLOAT_OF_BFP(imu_g, imu.accel);
  const float alpha = 0.92;
  ahrs_impl.lp_accel = alpha * ahrs_impl.lp_accel +
    (1. - alpha) *(FLOAT_VECT3_NORM(imu_g) - 9.81);
  const struct FloatVect3 earth_g = {0.,  0., -9.81 };
  const float dn = 250*fabs( ahrs_impl.lp_accel );
  const float g_noise[] = {1.+dn, 1.+dn, 1.+dn};
  //  const float g_noise[] = {150., 150., 150.};
  update_state(&earth_g, &imu_g, g_noise);
  reset_state();
}


void ahrs_update_mag(void) {
  struct FloatVect3 imu_h;
  MAGS_FLOAT_OF_BFP(imu_h, imu.mag);
  const struct FloatVect3 earth_h = { AHRS_H_X , AHRS_H_Y,  AHRS_H_Z };
  const float h_noise[] =  { 0.1610,  0.1771, 0.2659};
  update_state(&earth_h, &imu_h, h_noise);
  reset_state();
}


static inline void propagate_ref(void) {
  /* converts gyro to floating point */
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro_prev);

  /* unbias measurement */
  RATES_SUB(gyro_float, ahrs_impl.gyro_bias);

#ifdef AHRS_PROPAGATE_LOW_PASS_RATES
  /* lowpass angular rates */
  const float alpha = 0.1;
  FLOAT_RATES_LIN_CMB(ahrs_impl.imu_rate, ahrs_impl.imu_rate,
                      (1.-alpha), gyro_float, alpha);
#else
  RATES_COPY(ahrs_impl.imu_rate, gyro_float);
#endif


  /* propagate reference quaternion only if rate is non null */
  const float no = FLOAT_RATES_NORM(ahrs_impl.imu_rate);
  if (no > FLT_MIN) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    const float a  = 0.5*no*dt;
    const float ca = cosf(a);
    const float sa_ov_no = sinf(a)/no;
    const float dp = sa_ov_no*ahrs_impl.imu_rate.p;
    const float dq = sa_ov_no*ahrs_impl.imu_rate.q;
    const float dr = sa_ov_no*ahrs_impl.imu_rate.r;
    const float qi = ahrs_impl.ltp_to_imu_quat.qi;
    const float qx = ahrs_impl.ltp_to_imu_quat.qx;
    const float qy = ahrs_impl.ltp_to_imu_quat.qy;
    const float qz = ahrs_impl.ltp_to_imu_quat.qz;
    ahrs_impl.ltp_to_imu_quat.qi = ca*qi - dp*qx - dq*qy - dr*qz;
    ahrs_impl.ltp_to_imu_quat.qx = dp*qi + ca*qx + dr*qy - dq*qz;
    ahrs_impl.ltp_to_imu_quat.qy = dq*qi - dr*qx + ca*qy + dp*qz;
    ahrs_impl.ltp_to_imu_quat.qz = dr*qi + dq*qx - dp*qy + ca*qz;

    //    printf("%f\n",  ahrs_impl.ltp_to_imu_quat.qi);

    FLOAT_RMAT_OF_QUAT(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);
  }

}

/**
 * Progagate filter's covariance
 * We don't propagate state as we assume to have reseted
 */
static inline void propagate_state(void) {

  /* predict covariance */
  const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
  const float dp = ahrs_impl.imu_rate.p*dt;
  const float dq = ahrs_impl.imu_rate.q*dt;
  const float dr = ahrs_impl.imu_rate.r*dt;

  float F[6][6] = {{  1.,   dr,  -dq,  -dt,   0.,   0.  },
                   { -dr,   1.,   dp,   0.,  -dt,   0.  },
                   {  dq,  -dp,   1.,   0.,   0.,  -dt  },
                   {  0.,   0.,   0.,   1.,   0.,   0.  },
                   {  0.,   0.,   0.,   0.,   1.,   0.  },
                   {  0.,   0.,   0.,   0.,   0.,   1.  }};
  // P = FPF' + GQG
  float tmp[6][6];
  MAT_MUL(6,6,6, tmp, F, ahrs_impl.P);
  MAT_MUL_T(6,6,6,  ahrs_impl.P, tmp, F);
  const float dt2 = dt * dt;
  const float GQG[6] = {dt2*10e-3, dt2*10e-3, dt2*10e-3, dt2*9e-6, dt2*9e-6, dt2*9e-6 };
  for (int i=0;i<6;i++)
    ahrs_impl.P[i][i] += GQG[i];

}


/**
 *  Incorporate one 3D vector measurement
 */
static inline void update_state(const struct FloatVect3 *i_expected, struct FloatVect3* b_measured, const float noise[]) {

  /* converted expected measurement from inertial to body frame */
  struct FloatVect3 b_expected;
  FLOAT_RMAT_VECT3_MUL(b_expected, ahrs_impl.ltp_to_imu_rmat, *i_expected);

  // S = HPH' + JRJ
  float H[3][6] = {{           0., -b_expected.z,  b_expected.y, 0., 0., 0.},
                   { b_expected.z,            0., -b_expected.x, 0., 0., 0.},
                   {-b_expected.y,  b_expected.x,            0., 0., 0., 0.}};
  float tmp[3][6];
  MAT_MUL(3,6,6, tmp, H, ahrs_impl.P);
  float S[3][3];
  MAT_MUL_T(3,6,3, S, tmp, H);
  for (int i=0;i<3;i++)
    S[i][i] += noise[i];
  float invS[3][3];
  MAT_INV33(invS, S);

  // K = PH'invS
  float tmp2[6][3];
  MAT_MUL_T(6,6,3, tmp2, ahrs_impl.P, H);
  float K[6][3];
  MAT_MUL(6,3,3, K, tmp2, invS);

  // P = (I-KH)P
  float tmp3[6][6];
  MAT_MUL(6,3,6, tmp3, K, H);
  float I6[6][6] = {{ 1., 0., 0., 0., 0., 0. },
                    {  0., 1., 0., 0., 0., 0. },
                    {  0., 0., 1., 0., 0., 0. },
                    {  0., 0., 0., 1., 0., 0. },
                    {  0., 0., 0., 0., 1., 0. },
                    {  0., 0., 0., 0., 0., 1. }};
  float tmp4[6][6];
  MAT_SUB(6,6, tmp4, I6, tmp3);
  float tmp5[6][6];
  MAT_MUL(6,6,6, tmp5, tmp4, ahrs_impl.P);
  memcpy(ahrs_impl.P, tmp5, sizeof(ahrs_impl.P));

  // X = X + Ke
  struct FloatVect3 e;
  VECT3_DIFF(e, *b_measured, b_expected);
  ahrs_impl.gibbs_cor.qx  += K[0][0]*e.x + K[0][1]*e.y + K[0][2]*e.z;
  ahrs_impl.gibbs_cor.qy  += K[1][0]*e.x + K[1][1]*e.y + K[1][2]*e.z;
  ahrs_impl.gibbs_cor.qz  += K[2][0]*e.x + K[2][1]*e.y + K[2][2]*e.z;
  ahrs_impl.gyro_bias.p  += K[3][0]*e.x + K[3][1]*e.y + K[3][2]*e.z;
  ahrs_impl.gyro_bias.q  += K[4][0]*e.x + K[4][1]*e.y + K[4][2]*e.z;
  ahrs_impl.gyro_bias.r  += K[5][0]*e.x + K[5][1]*e.y + K[5][2]*e.z;

}


/**
 * Incorporate errors to reference and zeros state
 */
static inline void reset_state(void) {

  ahrs_impl.gibbs_cor.qi = 2.;
  struct FloatQuat q_tmp;
  FLOAT_QUAT_COMP(q_tmp, ahrs_impl.ltp_to_imu_quat, ahrs_impl.gibbs_cor);
  FLOAT_QUAT_NORMALIZE(q_tmp);
  memcpy(&ahrs_impl.ltp_to_imu_quat, &q_tmp, sizeof(ahrs_impl.ltp_to_imu_quat));
  FLOAT_QUAT_ZERO(ahrs_impl.gibbs_cor);
  FLOAT_RMAT_OF_QUAT(ahrs_impl.ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);

}

/**
 * Compute body orientation and rates from imu orientation and rates
 */
static inline void set_body_state_from_quat(void) {

  /* Compute LTP to BODY quaternion */
  struct FloatQuat ltp_to_body_quat;
  FLOAT_QUAT_COMP_INV(ltp_to_body_quat, ahrs_impl.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  /* Set in state interface */
  stateSetNedToBodyQuat_f(&ltp_to_body_quat);

  /* compute body rates */
  struct FloatRates body_rate;
  FLOAT_RMAT_TRANSP_RATEMULT(body_rate, ahrs_impl.body_to_imu_rmat, ahrs_impl.imu_rate);
  /* Set state */
  stateSetBodyRates_f(&body_rate);

}
