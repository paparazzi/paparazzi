/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "math/pprz_algebra_float.h"

/* our estimated attitude                     */
struct FloatQuat  bafe_quat;
/* our estimated gyro biases                  */
struct FloatRates bafe_bias;
/* we get unbiased body rates as byproduct    */
struct FloatRates bafe_rates;
/* maintain a euler angles representation     */
struct FloatEulers bafe_eulers;
/* maintains a rotation matrix representation */
struct FloatEulers bafe_dcm;
/* time derivative of our quaternion */
struct FloatQuat bafe_qdot;

#define BAFE_SSIZE 7
/* covariance matrix and its time derivative */
float bafe_P[BAFE_SSIZE][BAFE_SSIZE];
float bafe_Pdot[BAFE_SSIZE][BAFE_SSIZE];

/*
 * F represents the Jacobian of the derivative of the system with respect
 * its states.  We do not allocate the bottom three rows since we know that
 * the derivatives of bias_dot are all zero.
 */
float bafe_F[4][7];

/*
 * Kalman filter variables.
 */
float bafe_PHt[7];
float bafe_K[7];
float bafe_E;
/*
 * H represents the Jacobian of the measurements of the attitude
 * with respect to the states of the filter.  We do not allocate the bottom
 * three rows since we know that the attitude measurements have no
 * relationship to gyro bias.
 */
float bafe_H[4];
/* temporary variable used during state covariance update */
float bafe_FP[4][7];

/*
 * Q is our estimate noise variance.  It is supposed to be an NxN
 * matrix, but with elements only on the diagonals.  Additionally,
 * since the quaternion has no expected noise (we can't directly measure
 * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
 * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
 */
/* I measured about 0.009 rad/s noise */
#define bafe_Q_gyro 8e-03

/*
 * R is our measurement noise estimate.  Like Q, it is supposed to be
 * an NxN matrix with elements on the diagonals.  However, since we can
 * not directly measure the gyro bias, we have no estimate for it.
 * We only have an expected noise in the pitch and roll accelerometers
 * and in the compass.
 */
#define BAFE_R_PHI   1.3 * 1.3
#define BAFE_R_THETA 1.3 * 1.3
#define BAFE_R_PSI   2.5 * 2.5

#ifndef BAFE_DT
#define BAFE_DT (1./512.)
#endif

extern void ahrs_init(void);
extern void ahrs_align(void);


/*
 * Propagate our dynamic system
 *
 *      quat_dot = Wxq(pqr) * quat
 *      bias_dot = 0
 *
 * Wxq is the quaternion omega matrix:
 *
 *              [ 0, -p, -q, -r ]
 *      1/2 *   [ p,  0,  r, -q ]
 *              [ q, -r,  0,  p ]
 *              [ r,  q, -p,  0 ]
 *
 *
 *
 *
 *                 [ 0  -p  -q  -r   q1  q2  q3]
 *   F =   1/2 *   [ p   0   r  -q  -q0  q3 -q2]
 *                 [ q  -r   0   p  -q3 -q0  q1]
 *                 [ r   q  -p   0   q2 -q1 -q0]
 *                 [ 0   0   0   0    0   0   0]
 *                 [ 0   0   0   0    0   0   0]
 *                 [ 0   0   0   0    0   0   0]
 *
 */

void ahrs_propagate(void) {
  /* compute unbiased rates */
  RATES_FLOAT_OF_BFP(bafe_rates, imu.gyro);
  RATES_SUB(bafe_rates, bafe_bias);

  /* compute F
     F is only needed later on to update the state covariance P.
     However, its [0:3][0:3] region is actually the Wxq(pqr) which is needed to
     compute the time derivative of the quaternion, so we compute F now
  */

  /* Fill in Wxq(pqr) into F */
  bafe_F[0][0] = bafe_F[1][1] = bafe_F[2][2] = bafe_F[3][3] = 0;
  bafe_F[1][0] = bafe_F[2][3] = bafe_rates.p * 0.5;
  bafe_F[2][0] = bafe_F[3][1] = bafe_rates.q * 0.5;
  bafe_F[3][0] = bafe_F[1][2] = bafe_rates.r * 0.5;

  bafe_F[0][1] = bafe_F[3][2] = -bafe_F[1][0];
  bafe_F[0][2] = bafe_F[1][3] = -bafe_F[2][0];
  bafe_F[0][3] = bafe_F[2][1] = -bafe_F[3][0];
  /* Fill in [4:6][0:3] region */
  bafe_F[0][4] = bafe_F[2][6] = bafe_quat.qx * 0.5;
  bafe_F[0][5] = bafe_F[3][4] = bafe_quat.qy * 0.5;
  bafe_F[0][6] = bafe_F[1][5] = bafe_quat.qz * 0.5;

  bafe_F[1][4] =  bafe_F[2][5] = bafe_F[3][6] = bafe_quat.qi * -0.5;
  bafe_F[3][5] = -bafe_F[0][4];
  bafe_F[1][6] = -bafe_F[0][5];
  bafe_F[2][4] = -bafe_F[0][6];
  /* quat_dot = Wxq(pqr) * quat */
  bafe_qdot.qi=                           bafe_F[0][1]*bafe_quat.qx+bafe_F[0][2]*bafe_quat.qy+bafe_F[0][3] * bafe_quat.qz;
  bafe_qdot.qx= bafe_F[1][0]*bafe_quat.qi                          +bafe_F[1][2]*bafe_quat.qy+bafe_F[1][3] * bafe_quat.qz;
  bafe_qdot.qy= bafe_F[2][0]*bafe_quat.qi+bafe_F[2][1]*bafe_quat.qx                          +bafe_F[2][3] * bafe_quat.qz;
  bafe_qdot.qz= bafe_F[3][0]*bafe_quat.qi+bafe_F[3][1]*bafe_quat.qx+bafe_F[3][2]*bafe_quat.qy                            ;
  /* propagate quaternion */
  bafe_quat.qi += bafe_qdot.qi * BAFE_DT;
  bafe_quat.qx += bafe_qdot.qx * BAFE_DT;
  bafe_quat.qy += bafe_qdot.qy * BAFE_DT;
  bafe_quat.qz += bafe_qdot.qz * BAFE_DT;


}


extern void ahrs_update(void);




#endif /* AHRS_FLOAT_EKF_H */
