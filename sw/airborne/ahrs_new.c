
#include "ahrs_new.h"

#include <math.h>
#include <inttypes.h>
#include <string.h>

#include "frames.h"

#define C_PI		((real_t) 3.14159265358979323846264338327950)

/*
 * dt is our time step.  It is important to be close to the actual
 * frequency with which ahrs_state_update() is called with the body
 * angular rates.
 */
//#define dt              0.04
//#define dt              0.0625
//#define dt                0.0117185
#define dt                0.015625

//#define CONFIG_SPLIT_COVARIANCE

#ifdef CONFIG_SPLIT_COVARIANCE
#define dt_covariance   2 * dt
#else
#define dt_covariance   dt
#endif

/*
 * We have seven variables in our state -- the quaternion attitude
 * estimate and three gyro bias values.  The state time update equation
 * comes from the IMU gyro sensor readings:
 *
 *      Q_dot           = Wxq(pqr) * Q
 *      Bias_dot        = 0
 */

real_t q0, q1, q2, q3;
real_t bias_p, bias_q, bias_r;


/*
 * The user inputs the acceleration and rotational rates here, then
 * calls the update functions.
 */
real_t  ahrs_pqr[3];
//int16_t accel[3];


/*
 * The euler estimate will be updated less frequently than the
 * quaternion, but some applications are ok with that.
 */
real_t  ahrs_euler[3];


/*
 * The Direction Cosine Matrix is used to help rotate measurements
 * to and from the body frame.  We only need five elements from it,
 * so those are computed explicitly rather than the entire matrix
 *
 * External routines might want these (to until sensor readings),
 * so we export them.
 */
static real_t           dcm00;
static real_t           dcm01;
static real_t           dcm02;
static real_t           dcm12;
static real_t           dcm22;

/*
 * Covariance matrix and covariance matrix derivative are updated
 * every other state step.  This is because the covariance should change
 * at a rate somewhat slower than the dynamics of the system.
 */
static real_t           P[7][7];
static real_t           Pdot[7][7];
#ifdef CONFIG_SPLIT_COVARIANCE
static index_t          covariance_state;
#endif

/*
 * A represents the Jacobian of the derivative of the system with respect
 * its states.  We do not allocate the bottom three rows since we know that
 * the derivatives of bias_dot are all zero.
 */
real_t           A[4][7];

/*
 * These Kalman filter variables share space with A when the filter
 * is being updated.
 */
real_t           PCt[7];
real_t           K[7];
real_t           E;

/*
 * C represents the Jacobian of the measurements of the attitude
 * with respect to the states of the filter.  We do not allocate the bottom
 * three rows since we know that the attitude measurements have no
 * relationship to gyro bias.
 *
 * Since we compute each axis independently, we only allocate one
 * column out of the C matrix at a time.  This allows us to reuse the
 * matrix.  Additionally, this space is shared by Qdot.
 */
real_t           C[4];
real_t           Qdot[4];


/*
 * Q is our estimate noise variance.  It is supposed to be an NxN
 * matrix, but with elements only on the diagonals.  Additionally,
 * since the quaternion has no expected noise (we can't directly measure
 * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
 * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
 */
//#define Q_gyro          0.0075
/* I measured about 0.009 rad/s noise */
#define Q_gyro          8e-05


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
//#define R_pitch         0.1 * 0.1
//#define R_roll          0.1 * 0.1
//#define R_yaw            0.5 * 0.5



/*
 * Simple helper to normalize our quaternion attitude estimate.  We could
 * probably do this at a lower time step to save on calls to sqrt().
 */
void
norm_quat( void )
{
  //        index_t         i;
  real_t mag = 0;

  //        for( i=0 ; i<4 ; i++ )
  //                mag += quat[i] * quat[i];
  mag = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  
  mag = sqrt( mag );

  //        for( i=0 ; i<4 ; i++ )
  //                quat[i] /= mag;
  q0 /= mag;
  q1 /= mag;
  q2 /= mag;
  q3 /= mag;

}

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
static inline void
compute_A_quat( void )
{
  /* Unbias our gyro values */

  /* Zero our A matrix since it is shared with K */
  memset( (void*) A, 0, sizeof( A ) );

  /* Fill in Wxq(pqr) into A */
  /* A[0][0] = A[1][1] = A[2][2] = A[3][3] = 0; */
  A[1][0] = A[2][3] = (ahrs_pqr[0] -= bias_p) * 0.5;
  A[2][0] = A[3][1] = (ahrs_pqr[1] -= bias_q) * 0.5;
  A[3][0] = A[1][2] = (ahrs_pqr[2] -= bias_r) * 0.5;

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
 * However, this takes a very long time to compute.  So we split
 * the multiplications and additions into two separate routines.
 *
 * The first of these zeros P_dot, computes part of (A*P + P*A_tranpose)
 * and adds in the parts of Q that are non-zero.  Note that we know that
 * A has three zero rows at the bottom, so we do not include those in our
 * math.
 */
static void
covariance_update_0( void )
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
	  const real_t A_i_k = A[i][k];

	  Pdot[i][j] += A_i_k * P[k][j];
	  Pdot[j][i] += P[j][k] * A_i_k;
	}

  /*
   * Add in the non-zero parts of Q.  The quaternion portions
   * are all zero, and all the gyros share the same value.
   */
  for( i=4 ; i<7 ; i++ )
    Pdot[i][i] += Q_gyro;
}


/*
 * The second part of the covariance update computes the inner portion
 * of Pdot, the 4x4 region that corresponds to the quaternion.  It also
 * updates the covariance matrix P.
 */
static void
covariance_update_1( void )
{
  index_t                 i;
  index_t                 j;
  index_t                 k;

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
void
ahrs_state_update( void )
{
#if 0
  index_t                 i;
  index_t                 j;
#endif
  
  compute_A_quat();
  memset( Qdot, 0, sizeof(Qdot) );
  
  /* Compute Q_dot = Wxq(pqr) * Q (storing temp in C) */
#if 0
  for( i=0 ; i<4 ; i++ )
    {
      for( j=0 ; j<4 ; j++ )
	{
	  if( i == j )
	    continue;
	  Qdot[i] += A[i][j] * quat[j];
	}
    }
#else
  Qdot[0] =                A[0][1] * q1 + A[0][2] * q2 + A[0][3] * q3;
  Qdot[1] = A[1][0] * q0                + A[1][2] * q2 + A[1][3] * q3;
  Qdot[2] = A[2][0] * q0 + A[2][1] * q1                + A[2][3] * q3;
  Qdot[3] = A[3][0] * q0 + A[3][1] * q1 + A[3][2] * q2;
#endif
  
  /* Compute Q = Q + Q_dot * dt */
#if 0
  for( i=0 ; i<4 ; i++ )
    quat[i] += Qdot[i] * dt;
#else
  q0 += Qdot[0] * dt;
  q1 += Qdot[1] * dt;
  q2 += Qdot[2] * dt;
  q3 += Qdot[3] * dt;
#endif
  /*
   * We would normally renormalize our quaternion, but we will
   * allow it to drift until the next kalman update.
   */
  //norm_quat();

#ifdef CONFIG_SPLIT_COVARIANCE
  /* Compute our split covariance update */
  if( covariance_state == 0 ) {
    covariance_state = 1;
    covariance_update_0();
  } 
  else {
    covariance_state = 0;
    covariance_update_1();
  }
#else
  covariance_update_0();
  covariance_update_1();
#endif
}

/*
 * Compute the five elements of the DCM that we use for our
 * rotations and Jacobians.  This is used by several other functions
 * to speedup their computations.
 */
static void
compute_DCM( void )
{
  dcm00 = 1.0-2*(q2*q2 + q3*q3);
  dcm01 =     2*(q1*q2 + q0*q3);
  dcm02 =     2*(q1*q3 - q0*q2);
  dcm12 =     2*(q2*q3 + q0*q1);
  dcm22 = 1.0-2*(q1*q1 + q2*q2);
}


/*
 * Compute the Jacobian of the measurements to the system states.
 * You must have already computed the DCM for the quaternion before
 * calling this function.
 */
static inline void
compute_dphi_dq( void )
{
  index_t                 i;

  const real_t phi_err =  2 / (dcm22*dcm22 + dcm12*dcm12);

  C[0] = (q1 * dcm22);
  C[1] = (q0 * dcm22 + 2 * q1 * dcm12);
  C[2] = (q3 * dcm22 + 2 * q2 * dcm12);
  C[3] = (q2 * dcm22);

  for( i=0 ; i<4 ; i++ )
    C[i] *= phi_err;
}

static inline void
compute_dtheta_dq( void )
{
  const real_t theta_err  = -2 / sqrt( 1 - dcm02*dcm02 );

  C[0] = -q2 * theta_err;
  C[1] =  q3 * theta_err;
  C[2] = -q0 * theta_err;
  C[3] =  q1 * theta_err;
}

static inline void
compute_dpsi_dq( void )
{
  index_t                 i;

  const real_t psi_err    =  2 / (dcm00*dcm00 + dcm01*dcm01);

  C[0] = (q3 * dcm00);
  C[1] = (q2 * dcm00);
  C[2] = (q1 * dcm00 + 2 * q2 * dcm01);
  C[3] = (q0 * dcm00 + 2 * q3 * dcm01);

  for( i=0 ; i<4 ; i++ )
    C[i] *= psi_err;
}



/*
 * Translate our quaternion attitude estimate into Euler angles.
 * This is expensive, so don't do it often.  You must have already
 * computed the DCM for the quaternion before calling.
 */
static inline void
compute_euler_roll( void )
{
  ahrs_euler[0] = atan2( dcm12, dcm22 );
}

static inline void
compute_euler_pitch( void )
{
  ahrs_euler[1] = -asin( dcm02 );
}

static inline void
compute_euler_heading( void )
{
  ahrs_euler[2] = atan2( dcm01, dcm00 );
}


/*
 * The Kalman filter will share space with A, since it will be
 * recomputed each timestep.
 */
static void
run_kalman(
	   const real_t    R_axis,
	   const real_t    error
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
#endif

  /*
   * We would normally normalize our quaternion here, but
   * instead we will allow our caller to do it
   */
  //norm_quat();
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
ahrs_roll_update(
		 real_t                  roll
		 )
{

  // Reuse the DCM and A matrix from the compass computations
  compute_DCM();
  compute_euler_roll();
  compute_dphi_dq();

  /*
   * Compute the error in our roll estimate and measurement.
   * This can be between -180 and +180 degrees.
   */
  roll -= ahrs_euler[0];
  if( roll < -C_PI )
    roll += 2 * C_PI;
  if( roll > C_PI )
    roll -= 2 * C_PI;

  run_kalman( R_roll, roll );
  norm_quat();
}

void
ahrs_pitch_update(
		  real_t                  pitch
		  )
{

  // Reuse DCM
  compute_DCM();
  compute_euler_pitch();
  compute_dtheta_dq();

  /*
   * Compute the error in our pitch estimate and measurement.
   * Pitch can be between -90 and +90 degrees, so wrap it around
   * for the shortest distance.
   */
  pitch -= ahrs_euler[1];
  if( pitch < -C_PI / 2 )
    pitch += C_PI;
  if( pitch > C_PI / 2 )
    pitch -= C_PI;

  run_kalman( R_pitch, pitch );

  // We'll normalize our quaternion here only
  norm_quat();
}


/*
 * The rotation matrix to rotate from NED frame to body frame without
 * rotating in the yaw axis is:
 *
 * [ 1      0         0    ] [ cos(Theta)  0  -sin(Theta) ]
 * [ 0  cos(Phi)  sin(Phi) ] [      0      1       0      ]
 * [ 0 -sin(Phi)  cos(Phi) ] [ sin(Theta)  0   cos(Theta) ]
 *
 * This expands to:
 *
 * [  cos(Theta)              0      -sin(Theta)         ]
 * [  sin(Phi)*sin(Theta)  cos(Phi)   sin(Phi)*cos(Theta)]
 * [  cos(Phi)*sin(Theta)  -sin(Phi)  cos(Phi)*cos(Theta)]
 *
 * However, to untilt the compass reading, we need to use the 
 * transpose of this matrix.
 *
 * [  cos(Theta)  sin(Phi)*sin(Theta)  cos(Phi)*sin(Theta) ]
 * [      0       cos(Phi)            -sin(Phi)            ]
 * [ -sin(Theta)  sin(Phi)*cos(Theta)  cos(Phi)*cos(Theta) ]
 *
 * Additionally,
 * since we already have the DCM computed for our current attitude,
 * we can short cut all of the trig. substituting
 * in from the definition of euler2quat and quat2euler, we have:
 *
 * [ cos(Theta)   -dcm12*dcm02   -dcm22*dcm02 ]
 * [    0          dcm22         -dcm12       ]
 * [ dcm02         dcm12          dcm22       ]
 *
 * As installed in my AHRS, the magnetomer is rotated 180 degrees
 * about the Z axis relative to the IMU.  Thus, we must negate
 * mag[0] and mag[1], while mag[2] is still positive.
 */
real_t ahrs_heading_of_mag( const int16_t* mag ) {
  const real_t    ctheta  = cos( ahrs_euler[1] );
  const real_t    mn = ctheta * mag[0]
    - (dcm12 * mag[1] + dcm22 * mag[2]) * dcm02 / ctheta;
  
  const real_t    me =
    (dcm22 * mag[1] - dcm12 * mag[2]) / ctheta;
  
  const real_t heading = -atan2( me, mn );

  //  const real_t heading = -atan2( mag[AXIS_Y], mag[AXIS_X] );
  return heading;
}

real_t ahrs_roll_of_accel( real_t* accel_cal ) {
  return atan2(accel_cal[AXIS_Y], accel_cal[AXIS_Z]);
}

real_t ahrs_pitch_of_accel( real_t* accel_cal) {			
  float g2 =					
    accel_cal[AXIS_X]*accel_cal[AXIS_X] +	
    accel_cal[AXIS_Y]*accel_cal[AXIS_Y] +	
    accel_cal[AXIS_Z]*accel_cal[AXIS_Z];
  return -asin( accel_cal[AXIS_X] / sqrt( g2 ) );     
}

/*
 * Call this with an untilted heading
 */
void
ahrs_compass_update(
		    real_t                  heading
		    )
{
  // Update the DCM since this will require
  compute_DCM();
  compute_euler_heading();
  compute_dpsi_dq();

  /*
   * Compute the error in our heading estimate and measurement.
   * Heading can be between -180 and +180 degrees, so we wrap
   * to find the shortest turn between the two.
   */
  heading -= ahrs_euler[2];
  if( heading < -C_PI )
    heading += 2 * C_PI;
  if( heading > C_PI )
    heading -= 2 * C_PI;

  run_kalman( R_yaw, heading );
  norm_quat();
}


static void
euler2quat( void )
{
  const real_t            phi     = ahrs_euler[0] / 2.0;
  const real_t            theta   = ahrs_euler[1] / 2.0;
  const real_t            psi     = ahrs_euler[2] / 2.0;

  const real_t            shphi0   = sin( phi );
  const real_t            chphi0   = cos( phi );

  const real_t            shtheta0 = sin( theta );
  const real_t            chtheta0 = cos( theta );

  const real_t            shpsi0   = sin( psi );
  const real_t            chpsi0   = cos( psi );

  q0 =  chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0;
  q1 = -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0;
  q2 =  chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0;
  q3 =  chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0;
}

/*
 * Initialize the AHRS state data and covariance matrix.  The user
 * should have filled in the pqr and accel vectors before calling this.
 * They should also have set euler[2] from an untilted compass reading
 * and the euler[0], euler[1] from the accel2roll / accel2pitch values.
 */
void
ahrs_init(
	  const int16_t *         mag
	  )
{
  index_t                 i;

  euler2quat();
  compute_DCM();

  ahrs_euler[2] = ahrs_heading_of_mag( mag );

  euler2quat();
  compute_DCM();

  /* Initialize the bias terms so the filter doesn't work as hard */
  bias_p = ahrs_pqr[0];
  bias_q = ahrs_pqr[1];
  bias_r = ahrs_pqr[2];

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
