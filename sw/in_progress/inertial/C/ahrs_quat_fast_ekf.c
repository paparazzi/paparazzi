#include "ahrs_quat_fast_ekf.h"

#include <math.h>
#include <inttypes.h>
#include <string.h>


/* Time step */
#define afe_dt 0.0166667

/* We have seven variables in our state -- the quaternion attitude
 * estimate and three gyro bias values
 */
FLOAT_T afe_q0, afe_q1, afe_q2, afe_q3;
FLOAT_T afe_bias_p, afe_bias_q, afe_bias_r;

/* We maintain unbiased rates. */
FLOAT_T afe_p, afe_q, afe_r;

/* We maintain eulers */
FLOAT_T afe_phi, afe_theta, afe_psi;

/* time derivative of state
 * we know that bias_p_dot = bias_q_dot = bias_r_dot = 0
 */
FLOAT_T afe_q0_dot, afe_q1_dot, afe_q2_dot, afe_q3_dot;

/*
 * The Direction Cosine Matrix is used to help rotate measurements
 * to and from the body frame.  We only need five elements from it,
 * so those are computed explicitly rather than the entire matrix
 *
 * External routines might want these (to until sensor readings),
 * so we export them.
 */
FLOAT_T afe_dcm00;
FLOAT_T afe_dcm01;
FLOAT_T afe_dcm02;
FLOAT_T afe_dcm12;
FLOAT_T afe_dcm22;

/*
 * Covariance matrix and covariance matrix derivative
 */
FLOAT_T afe_P[7][7];
FLOAT_T afe_Pdot[7][7];

/*
 * F represents the Jacobian of the derivative of the system with respect
 * its states.  We do not allocate the bottom three rows since we know that
 * the derivatives of bias_dot are all zero.
 */
FLOAT_T afe_F[4][7];

/*
 * Kalman filter variables.
 */
FLOAT_T afe_PHt[7];
FLOAT_T afe_K[7];
FLOAT_T afe_E;

/*
 * H represents the Jacobian of the measurements of the attitude
 * with respect to the states of the filter.  We do not allocate the bottom
 * three rows since we know that the attitude measurements have no
 * relationship to gyro bias.
 */
FLOAT_T afe_H[4];
/* temporary variable used during state covariance update */
FLOAT_T afe_FP[4][7];

/*
 * Q is our estimate noise variance.  It is supposed to be an NxN
 * matrix, but with elements only on the diagonals.  Additionally,
 * since the quaternion has no expected noise (we can't directly measure
 * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
 * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
 */
/* I measured about 0.009 rad/s noise */
#define afe_Q_gyro 8e-03

/*
 * R is our measurement noise estimate.  Like Q, it is supposed to be
 * an NxN matrix with elements on the diagonals.  However, since we can
 * not directly measure the gyro bias, we have no estimate for it.
 * We only have an expected noise in the pitch and roll accelerometers
 * and in the compass.
 */
#define AFE_R_PHI   1.3 * 1.3
#define AFE_R_THETA 1.3 * 1.3
#define AFE_R_PSI   2.5 * 2.5

#include "ahrs_quat_fast_ekf_utils.h"

/*
 * Call ahrs_state_update every dt seconds with the raw body frame angular
 * rates.  It updates the attitude state estimate via this function:
 *
 *      quat_dot = Wxq(pqr) * quat
 *      bias_dot = 0
 *
 * Since F also contains Wxq, we fill it in here and then reuse the computed
 * values.  This avoids the extra floating point math.
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
 *                 [ q  -r   0   p  -q3  q0  q1]
 *                 [ r   q  -p   0   q2 -q1 -q0]
 *                 [ 0   0   0   0    0   0   0]
 *                 [ 0   0   0   0    0   0   0]
 *                 [ 0   0   0   0    0   0   0]
 *
 */
void afe_predict( const FLOAT_T* gyro ) {
  /* Unbias our gyro values */
  afe_p = gyro[0] -  afe_bias_p;
  afe_q = gyro[1] -  afe_bias_q;
  afe_r = gyro[2] -  afe_bias_r;

  /* compute F
     F is only needed later on to update the state covariance P.
     However, its [0:3][0:3] region is actually the Wxq(pqr) which is needed to
     compute the time derivative of the quaternion, so we compute F now */

  /* Fill in Wxq(pqr) into F */
  afe_F[0][0] = afe_F[1][1] = afe_F[2][2] = afe_F[3][3] = 0;
  afe_F[1][0] = afe_F[2][3] = afe_p * 0.5;
  afe_F[2][0] = afe_F[3][1] = afe_q * 0.5;
  afe_F[3][0] = afe_F[1][2] = afe_r * 0.5;

  afe_F[0][1] = afe_F[3][2] = -afe_F[1][0];
  afe_F[0][2] = afe_F[1][3] = -afe_F[2][0];
  afe_F[0][3] = afe_F[2][1] = -afe_F[3][0];
  /* Fill in [4:6][0:3] region */
  afe_F[0][4] = afe_F[2][6] = afe_q1 * 0.5;
  afe_F[0][5] = afe_F[3][4] = afe_q2 * 0.5;
  afe_F[0][6] = afe_F[1][5] = afe_q3 * 0.5;

  afe_F[1][4] = afe_F[2][5] = afe_F[3][6] = afe_q0 * -0.5;
  afe_F[3][5] = -afe_F[0][4];
  afe_F[1][6] = -afe_F[0][5];
  afe_F[2][4] = -afe_F[0][6];

  /* update state */
  /* quat_dot = Wxq(pqr) * quat */
  afe_q0_dot =                        afe_F[0][1] * afe_q1 + afe_F[0][2] * afe_q2 + afe_F[0][3] * afe_q3;
  afe_q1_dot = afe_F[1][0] * afe_q0                        + afe_F[1][2] * afe_q2 + afe_F[1][3] * afe_q3;
  afe_q2_dot = afe_F[2][0] * afe_q0 + afe_F[2][1] * afe_q1                        + afe_F[2][3] * afe_q3;
  afe_q3_dot = afe_F[3][0] * afe_q0 + afe_F[3][1] * afe_q1 + afe_F[3][2] * afe_q2;

  /* quat = quat + quat_dot * dt */
  afe_q0 += afe_q0_dot * afe_dt;
  afe_q1 += afe_q1_dot * afe_dt;
  afe_q2 += afe_q2_dot * afe_dt;
  afe_q3 += afe_q3_dot * afe_dt;

  /* normalize quaternion */
  AFE_NORM_QUAT();
  /* */
  AFE_DCM_OF_QUAT();
  AFE_EULER_OF_DCM();

#ifdef EKF_UPDATE_DISCRETE
  /*
   * update covariance
   * Pdot = F*P*F' + Q
   * P += Pdot * dt
   */
  uint8_t i, j, k;
  for (i=0; i<4; i++) {
    for (j=0; j<7; j++) {
      afe_FP[i][j] = 0.;
      for (k=0; k<7; k++) {
	afe_FP[i][j] += afe_F[i][k] * afe_P[k][j];
      }
    }
  }
  for (i=0; i<4; i++) {
    for (j=0; j<4; j++) {
      afe_Pdot[i][j] = 0.;
      for (k=0; k<7; k++) {
	afe_Pdot[i][j] += afe_FP[i][k] * afe_F[j][k];
      }
    }
  }
  /* add Q !!! added below*/
  //  Pdot[4][4] = afe_Q_gyro;
  //  Pdot[5][5] = afe_Q_gyro;
  //  Pdot[6][6] = afe_Q_gyro;

  /* P = P + Pdot * dt */
  for (i=0; i<4; i++)
    for (j=0; j<4; j++)
      afe_P[i][j] += afe_Pdot[i][j] * afe_dt;
  afe_P[4][4] += afe_Q_gyro * afe_dt;
  afe_P[5][5] += afe_Q_gyro * afe_dt;
  afe_P[6][6] += afe_Q_gyro * afe_dt;
#endif

#ifdef EKF_UPDATE_CONTINUOUS
  /* Pdot = F*P + F'*P + Q */
  memset( afe_Pdot, 0, sizeof(afe_Pdot) );
  /*
   * Compute the A*P+P*At for the bottom rows of P and A_tranpose
   */
  uint8_t i, j, k;
  for( i=0 ; i<4 ; i++ )
    for( j=0 ; j<7 ; j++ )
      for( k=4 ; k<7 ; k++ )
        {
          const FLOAT_T F_i_k = afe_F[i][k];
          afe_Pdot[i][j] += F_i_k * afe_P[k][j];
          afe_Pdot[j][i] += afe_P[j][k] * F_i_k;
        }
  /*
   * Compute F*P + P*Ft for the region [0..3][0..3]
   */
  for( i=0 ; i<4 ; i++ )
    for( j=0 ; j<4 ; j++ )
      for( k=0 ; k<4 ; k++ )
        {
          /* The diagonals of A are zero */
          if( i == k && j == k )
            continue;
          if( j == k )
            afe_Pdot[i][j] += afe_F[i][k] * afe_P[k][j];
          else
            if( i == k )
              afe_Pdot[i][j] += afe_P[i][k] * afe_F[j][k];
            else
              afe_Pdot[i][j] += ( afe_F[i][k] * afe_P[k][j] +
				  afe_P[i][k] * afe_F[j][k] );
        }
  /* Add in the non-zero parts of Q.  The quaternion portions
   * are all zero, and all the gyros share the same value.
   */
  for( i=4 ; i<7 ; i++ )
    afe_Pdot[i][i] += afe_Q_gyro;

 /* Compute P = P + Pdot * dt */
  for( i=0 ; i<7 ; i++ )
    for( j=0 ; j<7 ; j++ )
      afe_P[i][j] += afe_Pdot[i][j] * afe_dt;
#endif
}


/*
 *
 */
static void run_kalman( const FLOAT_T R_axis, const FLOAT_T error ) {
  int i, j;

  /* PHt = P * H' */
  afe_PHt[0] = afe_P[0][0] * afe_H[0] + afe_P[0][1] * afe_H[1] + afe_P[0][2] * afe_H[2] + afe_P[0][3] * afe_H[3];
  afe_PHt[1] = afe_P[1][0] * afe_H[0] + afe_P[1][1] * afe_H[1] + afe_P[1][2] * afe_H[2] + afe_P[1][3] * afe_H[3];
  afe_PHt[2] = afe_P[2][0] * afe_H[0] + afe_P[2][1] * afe_H[1] + afe_P[2][2] * afe_H[2] + afe_P[2][3] * afe_H[3];
  afe_PHt[3] = afe_P[3][0] * afe_H[0] + afe_P[3][1] * afe_H[1] + afe_P[3][2] * afe_H[2] + afe_P[3][3] * afe_H[3];
  afe_PHt[4] = afe_P[4][0] * afe_H[0] + afe_P[4][1] * afe_H[1] + afe_P[4][2] * afe_H[2] + afe_P[4][3] * afe_H[3];
  afe_PHt[5] = afe_P[5][0] * afe_H[0] + afe_P[5][1] * afe_H[1] + afe_P[5][2] * afe_H[2] + afe_P[5][3] * afe_H[3];
  afe_PHt[6] = afe_P[6][0] * afe_H[0] + afe_P[6][1] * afe_H[1] + afe_P[6][2] * afe_H[2] + afe_P[6][3] * afe_H[3];

  /*  E = H * PHt + R */
  afe_E = R_axis;
  afe_E += afe_H[0] * afe_PHt[0];
  afe_E += afe_H[1] * afe_PHt[1];
  afe_E += afe_H[2] * afe_PHt[2];
  afe_E += afe_H[3] * afe_PHt[3];

  /* Compute the inverse of E */
  afe_E = 1.0 / afe_E;

  /* Compute K = P * H' * inv(E) */
  afe_K[0] = afe_PHt[0] * afe_E;
  afe_K[1] = afe_PHt[1] * afe_E;
  afe_K[2] = afe_PHt[2] * afe_E;
  afe_K[3] = afe_PHt[3] * afe_E;
  afe_K[4] = afe_PHt[4] * afe_E;
  afe_K[5] = afe_PHt[5] * afe_E;
  afe_K[6] = afe_PHt[6] * afe_E;

  /* Update our covariance matrix: P = P - K * H * P */

  /* Compute HP = H * P, reusing the PHt array. */
  afe_PHt[0] = afe_H[0] * afe_P[0][0] + afe_H[1] * afe_P[1][0] + afe_H[2] * afe_P[2][0] + afe_H[3] * afe_P[3][0];
  afe_PHt[1] = afe_H[0] * afe_P[0][1] + afe_H[1] * afe_P[1][1] + afe_H[2] * afe_P[2][1] + afe_H[3] * afe_P[3][1];
  afe_PHt[2] = afe_H[0] * afe_P[0][2] + afe_H[1] * afe_P[1][2] + afe_H[2] * afe_P[2][2] + afe_H[3] * afe_P[3][2];
  afe_PHt[3] = afe_H[0] * afe_P[0][3] + afe_H[1] * afe_P[1][3] + afe_H[2] * afe_P[2][3] + afe_H[3] * afe_P[3][3];
  afe_PHt[4] = afe_H[0] * afe_P[0][4] + afe_H[1] * afe_P[1][4] + afe_H[2] * afe_P[2][4] + afe_H[3] * afe_P[3][4];
  afe_PHt[5] = afe_H[0] * afe_P[0][5] + afe_H[1] * afe_P[1][5] + afe_H[2] * afe_P[2][5] + afe_H[3] * afe_P[3][5];
  afe_PHt[6] = afe_H[0] * afe_P[0][6] + afe_H[1] * afe_P[1][6] + afe_H[2] * afe_P[2][6] + afe_H[3] * afe_P[3][6];

  /* Compute P -= K * HP (aliased to PHt) */
  for( i=0 ; i<4 ; i++ )
    for( j=0 ; j<7 ; j++ )
      afe_P[i][j] -= afe_K[i] * afe_PHt[j];

  /* Update our state: X += K * error */
  afe_q0     += afe_K[0] * error;
  afe_q1     += afe_K[1] * error;
  afe_q2     += afe_K[2] * error;
  afe_q3     += afe_K[3] * error;
  afe_bias_p += afe_K[4] * error;
  afe_bias_q += afe_K[5] * error;
  afe_bias_r += afe_K[6] * error;

  /* normalize quaternion */
  AFE_NORM_QUAT();
}


/*
 * Do the Kalman filter on the acceleration and compass readings.
 * This is normally a very simple:
 *
 *      E = H * P * H' + R
 *      K = P * H' * inv(E)
 *      P = P - K * H * P
 *      X = X + K * error
 *
 * We notice that P * H' is used twice, so we can cache the
 * results of it.
 *
 * H represents the Jacobian of measurements to states, which we know
 * to only have the top four rows filled in since the attitude
 * measurement does not relate to the gyro bias.  This allows us to
 * ignore parts of PHt
 *
 * We also only process one axis at a time to avoid having to perform
 * the 3x3 matrix inversion.
 */

void afe_update_phi( const FLOAT_T* accel ) {
  AFE_COMPUTE_H_PHI();
  FLOAT_T accel_phi = afe_phi_of_accel(accel);
  FLOAT_T err_phi = accel_phi - afe_phi;
  AFE_WARP( err_phi, M_PI);
  run_kalman( AFE_R_PHI, err_phi );
  AFE_DCM_OF_QUAT();
  AFE_EULER_OF_DCM();
}

void afe_update_theta( const FLOAT_T* accel ) {
  AFE_COMPUTE_H_THETA();
  FLOAT_T accel_theta = afe_theta_of_accel(accel);
  FLOAT_T err_theta = accel_theta - afe_theta;
  AFE_WARP( err_theta, M_PI_2);
  run_kalman( AFE_R_THETA, err_theta );
  AFE_DCM_OF_QUAT();
  AFE_EULER_OF_DCM();
}

void afe_update_psi( const int16_t* mag ) {
  AFE_COMPUTE_H_PSI();
  FLOAT_T mag_psi = afe_psi_of_mag(mag);
  FLOAT_T err_psi = mag_psi - afe_psi;
  AFE_WARP( err_psi, M_PI);
  run_kalman( AFE_R_PSI, err_psi );
  AFE_DCM_OF_QUAT();
  AFE_EULER_OF_DCM();
}

/*
 * Initialize the AHRS state data and covariance matrix.
 */
void afe_init( const int16_t *mag, const FLOAT_T* accel, const FLOAT_T* gyro ) {

  /* F and P will be updated only on the non zero locations */
  memset( (void*) afe_F, 0, sizeof( afe_F ) );
  memset( (void*) afe_P, 0, sizeof( afe_P ) );
  int i;
  for( i=0 ; i<4 ; i++ )
    afe_P[i][i] = 1.;
  for( i=4 ; i<7 ; i++ )
    afe_P[i][i] = .5;

  /* assume vehicle is still, so initial bias are gyro readings */
  afe_bias_p = gyro[0];
  afe_bias_q = gyro[1];
  afe_bias_r = gyro[2];

  afe_phi = afe_phi_of_accel(accel);
  afe_theta = afe_theta_of_accel(accel);
  afe_psi = 0.;

  AFE_QUAT_OF_EULER();
  AFE_DCM_OF_QUAT();
  afe_psi = afe_psi_of_mag( mag );

  AFE_QUAT_OF_EULER();
  AFE_DCM_OF_QUAT();


}
