
#include "std.h"
#include "ahrs_new.h"
#include "ahrs_utils.h"

#include <math.h>
#include <inttypes.h>
#include <string.h>

#include "frames.h"

/*
 * dt is our time step.  It is important to be close to the actual
 * frequency with which ahrs_state_update() is called with the body
 * angular rates.
 */
#define dt              0.015625
#define dt_covariance   dt


/*
 * We have seven variables in our state -- the quaternion attitude
 * estimate and three gyro bias values.  The state time update equation
 * comes from the IMU gyro sensor readings:
 *
 *      Q_dot           = Wxq(pqr) * Q
 *      Bias_dot        = 0
 */

float q0, q1, q2, q3;
float bias_p, bias_q, bias_r;

/*
 * We maintain eulers.
 */
float ahrs_phi, ahrs_theta, ahrs_psi;

/*
 * We maintain unbiased rates.
 */
float ahrs_p, ahrs_q, ahrs_r;

/*
 * The Direction Cosine Matrix is used to help rotate measurements
 * to and from the body frame.  We only need five elements from it,
 * so those are computed explicitly rather than the entire matrix
 *
 * External routines might want these (to until sensor readings),
 * so we export them.
 */
static float           dcm00;
static float           dcm01;
static float           dcm02;
static float           dcm12;
static float           dcm22;

/*
 * Covariance matrix and covariance matrix derivative are updated
 * every other state step.  This is because the covariance should change
 * at a rate somewhat slower than the dynamics of the system.
 */
float        P[7][7];
static float Pdot[7][7];

/*
 * A represents the Jacobian of the derivative of the system with respect
 * its states.  We do not allocate the bottom three rows since we know that
 * the derivatives of bias_dot are all zero.
 */
float           A[4][7];

/*
 * Kalman filter variables.
 */
float           PCt[7];
float           K[7];
float           E;

/*
 * C represents the Jacobian of the measurements of the attitude
 * with respect to the states of the filter.  We do not allocate the bottom
 * three rows since we know that the attitude measurements have no
 * relationship to gyro bias.
 *
 * Since we compute each axis independently, we only allocate one
 * column out of the C matrix at a time.  This allows us to reuse the
 * matrix.
 */
float           C[4];
float           Qdot[4];


/*
 * Q is our estimate noise variance.  It is supposed to be an NxN
 * matrix, but with elements only on the diagonals.  Additionally,
 * since the quaternion has no expected noise (we can't directly measure
 * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
 * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
 */
//#define Q_gyro          0.0075
/* I measured about 0.009 rad/s noise */
#define Q_gyro          8e-03


/*
 * R is our measurement noise estimate.  Like Q, it is supposed to be
 * an NxN matrix with elements on the diagonals.  However, since we can
 * not directly measure the gyro bias, we have no estimate for it.
 * We only have an expected noise in the pitch and roll accelerometers
 * and in the compass.
 */
#define R_pitch         1.3 * 1.3
#define R_roll          1.3 * 1.3
#define R_yaw           2.5 * 2.5

/*
 * These are the rows and columns of A that relate the derivative
 * of the quaternion to the quaternion.  It is actual the omega
 * cross matrix of the body rates in this case.
 *
 * Wxq is the quaternion omega matrix:
 *
 *              [ 0, -p, -q, -r ]
 *      1/2 *   [ p,  0,  r, -q ]
 *              [ q, -r,  0,  p ]
 *              [ r,  q, -p,  0 ]
 *
 * Call compute_A_quat() before calling compute_A_bias
 */
static inline void compute_A_quat( const float* gyro )
{
  /* Unbias our gyro values */
  ahrs_p = gyro[0] -  bias_p;
  ahrs_q = gyro[1] -  bias_q;
  ahrs_r = gyro[2] -  bias_r;

  /* Zero our A matrix since it is shared with K */
  memset( (void*) A, 0, sizeof( A ) );

  /* Fill in Wxq(pqr) into A */
  /* A[0][0] = A[1][1] = A[2][2] = A[3][3] = 0; */
  A[1][0] = A[2][3] = ahrs_p * 0.5;
  A[2][0] = A[3][1] = ahrs_q * 0.5;
  A[3][0] = A[1][2] = ahrs_r * 0.5;

  A[0][1] = A[3][2] = -A[1][0];
  A[0][2] = A[1][3] = -A[2][0];
  A[0][3] = A[2][1] = -A[3][0];
}

/*
 * Finish filling in the terms of the Jacobian matrix A.  It has already
 * had the terms that relate the derivative of the quaternion to the
 * quaternion; this is now the terms that relate the derivative of the
 * quaternion to the gyro bias.  As mentioned above, the gyro bias
 * derivative is zero, so there is no need to fill in the bottom rows.
 *
 * Since the function for Q_dot = quatW( pqr - gyro_bias ) * Q,
 * we can compute these terms:
 *
 *      [  q1  q2  q3 ]
 *      [ -q0  q3 -q2 ] * 0.5
 *      [ -q3 -q0  q1 ]
 *      [  q2 -q1 -q0 ]
 */
static inline void
compute_A_bias( void )
{
  A[0][4] = A[2][6] = q1 * 0.5;
  A[0][5] = A[3][4] = q2 * 0.5;
  A[0][6] = A[1][5] = q3 * 0.5;

  A[1][4] = A[2][5] = A[3][6] = q0 * -0.5;
  A[3][5] = -A[0][4];
  A[1][6] = -A[0][5];
  A[2][4] = -A[0][6];
}


/*
 * Typically the covariance update equation is:
 *
 *      P_dot = A*P + P*A_transpose + Q
 *
 * The first of these zeros P_dot, computes part of (A*P + P*A_tranpose)
 * and adds in the parts of Q that are non-zero.  Note that we know that
 * A has three zero rows at the bottom, so we do not include those in our
 * math.
 *
 * The second part of the covariance update computes the inner portion
 * of Pdot, the 4x4 region that corresponds to the quaternion.  It also
 * updates the covariance matrix P.
 */
static void
covariance_update( void )
{
  index_t         i;
  index_t         j;
  index_t         k;

  memset( Pdot, 0, sizeof(Pdot) );

  /* Finish the computation of A */
  compute_A_bias();

  /*
   * Compute the A*P+P*At for the bottom rows of P and A_tranpose
   */
  for( i=0 ; i<4 ; i++ )
    for( j=0 ; j<7 ; j++ )
      for( k=4 ; k<7 ; k++ )
	{
	  const float A_i_k = A[i][k];

	  Pdot[i][j] += A_i_k * P[k][j];
	  Pdot[j][i] += P[j][k] * A_i_k;
	}

  /*
   * Add in the non-zero parts of Q.  The quaternion portions
   * are all zero, and all the gyros share the same value.
   */
  for( i=4 ; i<7 ; i++ )
    Pdot[i][i] += Q_gyro;

  /*
   * Compute A*P + P*At for the region [0..3][0..3]
   */
  for( i=0 ; i<4 ; i++ )
    for( j=0 ; j<4 ; j++ )
      for( k=0 ; k<4 ; k++ )
	{
	  /* The diagonals of A are zero */
	  if( i == k && j == k )
	    continue;
	  if( j == k )
	    Pdot[i][j] += A[i][k] * P[k][j];
	  else
	    if( i == k )
	      Pdot[i][j] += P[i][k] * A[j][k];
	    else
	      Pdot[i][j] += A[i][k] * P[k][j]
		+  P[i][k] * A[j][k];
	}

  /* Compute P = P + Pdot * dt */
  for( i=0 ; i<7 ; i++ )
    for( j=0 ; j<7 ; j++ )
      P[i][j] += Pdot[i][j] * dt_covariance;
}


/*
 * Call ahrs_state_update every dt seconds with the raw body frame angular
 * rates.  It updates the attitude state estimate via this function:
 *
 *      Q_dot = Wxq(pqr) * Q
 *
 * Since A also contains Wxq, we fill it in here and then reuse the computed
 * values.  This avoids the extra floating point math.
 *
 * Wxq is the quaternion omega matrix:
 *
 *              [ 0, -p, -q, -r ]
 *      1/2 *   [ p,  0,  r, -q ]
 *              [ q, -r,  0,  p ]
 *              [ r,  q, -p,  0 ]
 */
void ahrs_predict( const float* gyro ) {

  compute_A_quat( gyro );
  memset( Qdot, 0, sizeof(Qdot) );
  
  /* Compute Q_dot = Wxq(pqr) * Q (storing temp in C) */
  Qdot[0] =                A[0][1] * q1 + A[0][2] * q2 + A[0][3] * q3;
  Qdot[1] = A[1][0] * q0                + A[1][2] * q2 + A[1][3] * q3;
  Qdot[2] = A[2][0] * q0 + A[2][1] * q1                + A[2][3] * q3;
  Qdot[3] = A[3][0] * q0 + A[3][1] * q1 + A[3][2] * q2;
  
  /* Compute Q = Q + Q_dot * dt */
  q0 += Qdot[0] * dt;
  q1 += Qdot[1] * dt;
  q2 += Qdot[2] * dt;
  q3 += Qdot[3] * dt;

  /*
   * We would normally renormalize our quaternion, but we will
   * allow it to drift until the next kalman update.
   */
  norm_quat();

  covariance_update();
}

/*
 * Translate our quaternion attitude estimate into Euler angles.
 * This is expensive, so don't do it often.  You must have already
 * computed the DCM for the quaternion before calling.
 */
static inline void
compute_euler_roll( void )
{
  ahrs_phi = atan2( dcm12, dcm22 );
}

static inline void
compute_euler_pitch( void )
{
  ahrs_theta = -asin( dcm02 );
}

static inline void
compute_euler_heading( void )
{
  ahrs_psi = atan2( dcm01, dcm00 );
}


/*
 * The Kalman filter will share space with A, since it will be
 * recomputed each timestep.
 */
static void
run_kalman(
	   const float    R_axis,
	   const float    error
	   )
{
  index_t         i;
  index_t         j;
  index_t         k;

  memset( (void*) PCt, 0, sizeof( PCt ) );

  /* Compute PCt = P * C_tranpose */
  for( i=0 ; i<7 ; i++ )
    for( k=0 ; k<4 ; k++ )
      PCt[i] += P[i][k] * C[k];

  /* Compute E = C * PCt + R */
  E = R_axis;
  for( i=0 ; i<4 ; i++ )
    E += C[i] * PCt[i];

  /* Compute the inverse of E */
  E = 1.0 / E;

  /* Compute K = P * C_tranpose * inv(E) */
  for( i=0 ; i<7 ; i++ )
    K[i] = PCt[i] * E;

  /* Update our covariance matrix: P = P - K * C * P */

  /* Compute CP = C * P, reusing the PCt array. */
  memset( (void*) PCt, 0, sizeof( PCt ) );
  for( j=0 ; j<7 ; j++ )
    for( k=0 ; k<4 ; k++ )
      PCt[j] += C[k] * P[k][j];

  /* Compute P -= K * CP (aliased to PCt) */
  for( i=0 ; i<7 ; i++ )
    for( j=0 ; j<7 ; j++ )
      P[i][j] -= K[i] * PCt[j];

  /* Update our state: X += K * error */
#if 0
  for( i=0 ; i<7 ; i++ )
    X[i] += K[i] * error;
#else
  q0     += K[0] * error;
  q1     += K[1] * error;
  q2     += K[2] * error;
  q3     += K[3] * error;
  bias_p += K[4] * error;
  bias_q += K[5] * error;
  bias_r += K[6] * error;

/*   Bound(bias_p, -0.1, 0.1); */
/*   Bound(bias_q, -0.1, 0.1); */
/*   Bound(bias_r, -0.1, 0.1); */
#endif

  /*
   * We would normally normalize our quaternion here, but
   * instead we will allow our caller to do it
   */
  //  norm_quat();
}


/*
 * Do the Kalman filter on the acceleration and compass readings.
 * This is normally a very simple:
 *
 *      E = C * P * C_tranpose + R
 *      K = P * C_tranpose * inv(E)
 *      P = P - K * C * P
 *      X = X + K * error
 *
 * However, this would take forever.  Notably, the inv(E) routine
 * might be very time consuming, even for the 3x3 matrix that results
 * from our three measurements.
 *
 * We notice that P * C_tranpose is used twice, so we can cache the
 * results of it.
 *
 * C represents the Jacobian of measurements to states, which we know
 * to only have the top four rows filled in since the attitude
 * measurement does not relate to the gyro bias.  This allows us to
 * ignore parts of PCt that are not
 *
 * We also only process one axis at a time to avoid having to perform
 * the 3x3 matrix inversion.
 */

void
ahrs_roll_update( const float* accel ) {
  float accel_roll = ahrs_roll_of_accel(accel);
  // Reuse the DCM and A matrix from the compass computations
  DCM_of_quat();
  compute_euler_roll();
  compute_dphi_dq();

  /*
   * Compute the error in our roll estimate and measurement.
   * This can be between -180 and +180 degrees.
   */
  accel_roll -= ahrs_phi;
  if( accel_roll < -M_PI )
    accel_roll += 2 * M_PI;
  if( accel_roll > M_PI )
    accel_roll -= 2 * M_PI;

  run_kalman( R_roll, accel_roll );
  norm_quat();
}

void
ahrs_pitch_update( const float* accel ) {
  float accel_pitch = ahrs_pitch_of_accel(accel);
  // Reuse DCM
  DCM_of_quat();
  compute_euler_pitch();
  compute_dtheta_dq();

  /*
   * Compute the error in our pitch estimate and measurement.
   * Pitch can be between -90 and +90 degrees, so wrap it around
   * for the shortest distance.
   */
  accel_pitch -= ahrs_theta;
  if( accel_pitch < -M_PI_2 )
    accel_pitch += M_PI;
  if( accel_pitch > M_PI_2 )
    accel_pitch -= M_PI;

  run_kalman( R_pitch, accel_pitch );

  // We'll normalize our quaternion here only
  norm_quat();
}

void
ahrs_yaw_update( const int16_t* mag ) {
 
  float mag_yaw = ahrs_yaw_of_mag(mag);
  // Update the DCM since this will require
  DCM_of_quat();
  compute_euler_heading();
  compute_dpsi_dq();

  /*
   * Compute the error in our heading estimate and measurement.
   * Heading can be between -180 and +180 degrees, so we wrap
   * to find the shortest turn between the two.
   */
  mag_yaw -= ahrs_psi;
  if( mag_yaw < -M_PI )
    mag_yaw += 2 * M_PI;
  if( mag_yaw > M_PI )
    mag_yaw -= 2 * M_PI;

  run_kalman( R_yaw, mag_yaw );
  norm_quat();
}

/*
 * Initialize the AHRS state data and covariance matrix.  The user
 * should have filled in the pqr and accel vectors before calling this.
 * They should also have set euler[2] from an untilted compass reading
 * and the euler[0], euler[1] from the accel2roll / accel2pitch values.
 */
void ahrs_init( const int16_t *mag, const float* accel, const float* gyro ) {

  ahrs_phi = ahrs_roll_of_accel(accel);
  ahrs_theta = ahrs_pitch_of_accel(accel);
  ahrs_psi = 0.;

  quat_of_eulers();
  DCM_of_quat();
  ahrs_psi = ahrs_yaw_of_mag( mag );

 //  imu_mag[AXIS_X] = 100; imu_mag[AXIS_Y] = 0;  imu_mag[AXIS_Z] = 0; 
  
  index_t                 i;


  quat_of_eulers();
  DCM_of_quat();

  /* Initialize the bias terms so the filter doesn't work as hard */
  bias_p = gyro[0];
  bias_q = gyro[1];
  bias_r = gyro[2];

  /*
   * Setup our covariance matrix
   * It is 1 in the diagonal elements for which Q is zero in the
   * same elements and vice versa.  Thus, only the quaternion
   * rows/columns have any entries.
   */
  memset( (void*) P, 0, sizeof( P ) );
  for( i=0 ; i<4 ; i++ )
    P[i][i] = 1;
}
