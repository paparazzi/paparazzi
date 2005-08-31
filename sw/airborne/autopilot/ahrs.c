/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id$
 *
 * Fast AHRS object almost ready for use on microcontrollers
 *
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2005 Jean-Pierre Dumont <flyingtuxATfreeDOTfr>
 */


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>
#include "timer.h"
#include "uart.h"
#include "string.h"
#include "adc.h"
#include "ahrs.h"
#include "autopilot.h"
#include "link_fbw.h"

#ifdef PNI_MAG
#include "pni.h"
#endif //PNI_MAG

#ifdef SECTION_IMU_ANALOG

#define C_PI		((real_t) 3.14159265358979323846264338327950)

/*
 * dt is our time step.  It is important to be close to the actual
 * frequency with which ahrs_state_update() is called with the body
 * angular rates.
 */
#define	dt		0.05  //0.04

#define CONFIG_SPLIT_COVARIANCE
#ifdef CONFIG_SPLIT_COVARIANCE
#define	dt_covariance	2 * dt
#else
#define dt_covariance	dt
#endif

//TODO this part of code should be generaed by xml configuration
//#define		IMU_ADC_ACCELX	5
//#define		IMU_ADC_ACCELX_ZERO	0x24A
//#define		IMU_ADC_ACCELX_SIGN	-

//#define		IMU_ADC_ACCELY	6
//#define		IMU_ADC_ACCELY_ZERO	0x280
//#define		IMU_ADC_ACCELY_SIGN	+

//#define		IMU_ADC_ACCELZ	4
//#define		IMU_ADC_ACCELZ_ZERO	0x210
//#define		IMU_ADC_ACCELZ_SIGN	-

//#define		IMU_ADC_ROLL_DOT	3
//#define		IMU_ADC_ROLL_DOT_SIGN	+

//#define		IMU_ADC_PITCH_DOT	2
//#define		IMU_ADC_PITCH_DOT_SIGN	-

//#define		IMU_ADC_YAW_DOT	7
//#define		IMU_ADC_YAW_DOT_SIGN	-

//#define			IMU_INIT_EULER_DOT_VARIANCE_MAX		2
//#define			IMU_INIT_EULER_DOT_NB_SAMPLES_MIN	10

//#define		GYRO_SCALE	(real_t) (0.9444 * 3.14159 / 180.0)
//#define		GYRO_ZERO	0x200


/*
 * paparazzi adc_buff
 */
static struct adc_buf buf_accelX;
static struct adc_buf buf_accelY;
static struct adc_buf buf_accelZ;



/*
 * We have seven variables in our state -- the quaternion attitude
 * estimate and three gyro bias values.  The state time update equation
 * comes from the IMU gyro sensor readings:
 *
 *	Q_dot		= Wxq(pqr) * Q
 *	Bias_dot	= 0
 *
 * The actual data segment in the bss is defined in ahrs_data.S
 * Most of these are aliases to each other.
 */


/*
 * The user inputs the acceleration and rotational rates here, then
 * calls the update functions.
 */
real_t	pqr[3];
int16_t	accel[3];


/*
 * The euler estimate will be updated less frequently than the
 * quaternion, but some applications are ok with that.
 */
real_t	ahrs_euler[3];


/*
 * The Direction Cosine Matrix is used to help rotate measurements
 * to and from the body frame.  We only need five elements from it,
 * so those are computed explicitly rather than the entire matrix
 *
 * External routines might want these (to until sensor readings),
 * so we export them.
 */
static real_t		dcm00;
static real_t		dcm01;
static real_t		dcm02;
static real_t		dcm12;
static real_t		dcm22;


/*
 * Covariance matrix and covariance matrix derivative are updated
 * every other state step.  This is because the covariance should change
 * at a rate somewhat slower than the dynamics of the system.
 */
static real_t		P[7][7];
static real_t		Pdot[7][7];
static index_t		covariance_state;


/*
 * A represents the Jacobian of the derivative of the system with respect
 * its states.  We do not allocate the bottom three rows since we know that
 * the derivatives of bias_dot are all zero.
 */
extern real_t		A[4][7];

/*
 * These Kalman filter variables share space with A when the filter
 * is being updated.
 */
extern real_t		PCt[7];
extern real_t		K[7];
extern real_t		E;

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
extern real_t		C[4];
extern real_t		Qdot[4];


/*
 * Q is our estimate noise variance.  It is supposed to be an NxN
 * matrix, but with elements only on the diagonals.  Additionally,
 * since the quaternion has no expected noise (we can't directly measure
 * it), those are zero.  For the gyro, we expect around 5 deg/sec noise,
 * which is 0.08 rad/sec.  The variance is then 0.08^2 ~= 0.0075.
 */
#define Q_gyro		0.0075


/*
 * R is our measurement noise estimate.  Like Q, it is supposed to be
 * an NxN matrix with elements on the diagonals.  However, since we can
 * not directly measure the gyro bias, we have no estimate for it.
 * We only have an expected noise in the pitch and roll accelerometers
 * and in the compass.
 */
#define R_pitch		1.3 * 1.3
#define R_roll		1.0 * 1.0
#define R_yaw		2.5 * 2.5



/*
 * Simple helper to normalize our quaternion attitude estimate.  We could
 * probably do this at a lower time step to save on calls to sqrt().
 */
void
norm_quat( void )
{
	index_t		i;
	real_t		mag = 0;

	for( i=0 ; i<4 ; i++ )
		mag += quat[i] * quat[i];

	mag = sqrt( mag );

	for( i=0 ; i<4 ; i++ )
		quat[i] /= mag;
}


/*
 * These are the rows and columns of A that relate the derivative
 * of the quaternion to the quaternion.  It is actual the omega
 * cross matrix of the body rates in this case.
 *
 * Wxq is the quaternion omega matrix:
 *
 *		[ 0, -p, -q, -r ]
 *	1/2 *	[ p,  0,  r, -q ]
 *		[ q, -r,  0,  p ]
 *		[ r,  q, -p,  0 ]
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
	A[1][0] = A[2][3] = (pqr[0] -= bias[0]) * 0.5;
	A[2][0] = A[3][1] = (pqr[1] -= bias[1]) * 0.5;
	A[3][0] = A[1][2] = (pqr[2] -= bias[2]) * 0.5;

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
 *	[  q1  q2  q3 ]
 *	[ -q0  q3 -q2 ] * 0.5
 *	[ -q3 -q0  q1 ]
 *	[  q2 -q1 -q0 ]
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
 *	P_dot = A*P + P*A_transpose + Q
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
	index_t		i;
	index_t		j;
	index_t		k;

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
	index_t			i;
	index_t			j;
	index_t			k;

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



void
covariance_update( void )
{
}


/*
 * Call ahrs_state_update every dt seconds with the raw body frame angular
 * rates.  It updates the attitude state estimate via this function:
 *
 * 	Q_dot = Wxq(pqr) * Q
 *
 * Since A also contains Wxq, we fill it in here and then reuse the computed
 * values.  This avoids the extra floating point math.
 *
 * Wxq is the quaternion omega matrix:
 *
 *		[ 0, -p, -q, -r ]
 *	1/2 *	[ p,  0,  r, -q ]
 *		[ q, -r,  0,  p ]
 *		[ r,  q, -p,  0 ]
 */
void
ahrs_state_update( void )
{
	index_t			i;
	index_t			j;

	compute_A_quat();
	memset( Qdot, 0, sizeof(Qdot) );

	/* Compute Q_dot = Wxq(pqr) * Q (storing temp in C) */
	for( i=0 ; i<4 ; i++ )
	{
		for( j=0 ; j<4 ; j++ )
		{
			if( i == j )
				continue;
			Qdot[i] += A[i][j] * quat[j];
		}
	}

	/* Compute Q = Q + Q_dot * dt */
	for( i=0 ; i<4 ; i++ )
		quat[i] += Qdot[i] * dt;

	/*
	 * We would normally renormalize our quaternion, but we will
	 * allow it to drift until the next kalman update.
	 */
	//norm_quat();

#ifdef CONFIG_SPLIT_COVARIANCE
	/* Compute our split covariance update */
	if( covariance_state == 0 )
	{
		covariance_state = 1;
		covariance_update_0();
		return;
	} else {
		covariance_state = 0;
		covariance_update_1();
		return;
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
	index_t			i;

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
	const real_t theta_err	= -2 / sqrt( 1 - dcm02*dcm02 );

	C[0] = -q2 * theta_err;
	C[1] =  q3 * theta_err;
	C[2] = -q0 * theta_err;
	C[3] =  q1 * theta_err;
}

static inline void
compute_dpsi_dq( void )
{
	index_t			i;

	const real_t psi_err	=  2 / (dcm00*dcm00 + dcm01*dcm01);

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
	const real_t	R_axis,
	const real_t	error
)
{
	index_t		i;
	index_t		j;
	index_t		k;

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
	for( i=0 ; i<7 ; i++ )
		X[i] += K[i] * error;

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
 * 	E = C * P * C_tranpose + R
 *	K = P * C_tranpose * inv(E)
 *	P = P - K * C * P
 *	X = X + K * error
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
	real_t			roll
)
{

	// Reuse the DCM and A matrix from the compass computations
	//compute_DCM();
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
}


void
ahrs_pitch_update(
	real_t			pitch
)
{

	// Reuse DCM 
	//compute_DCM();
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
 * [  cos(Theta)  sin(Phi)*sin(Theta)  cos(Phi)*sin(Theta) ]
 * [      0       cos(Phi)            -sin(Phi)            ]
 * [ -sin(Theta)  sin(Phi)*cos(Theta)  cos(Phi)*cos(Theta) ]
 *
 * However, to untilt the compass reading, but not rotate it,
 * we need to use the transpose of this matrix.  Additionally,
 * since we already have the DCM computed for our current attitude,
 * we can short cut all of the trig.  Transposing and substituting
 * in from the definition of euler2quat and quat2euler, we have:
 *
 * [    ?         -dcm12*dcm02   -dcm22*dcm02 ]
 * [    0          dcm22         -dcm12       ]
 * [ dcm02         dcm12          dcm22       ]
 *
 * As installed in my AHRS, the magnetomer is rotated 180 degrees
 * about the Z axis relative to the IMU.  Thus, we must negate
 * mag[0] and mag[1], while mag[2] is still positive.
 */
#ifdef PNI_MAG
real_t
untilt_compass(
	const int16_t *		mag
)

	const real_t	ctheta	= cos( ahrs_euler[1] );
	const real_t	mn = ctheta * mag[0]
		- (dcm12 * mag[1] + dcm22 * mag[2]) * dcm02 / ctheta;

	const real_t	me =
		(dcm22 * mag[1] - dcm12 * mag[2]) / ctheta;

	const real_t	heading = -atan2( me, -mn );

	return heading;
}
#else
real_t
untilt_compass(
	const int16_t *		mag
)
{
    	//Faking Compass
	return 0;
}
#endif //PNI_MAG



/*
 * Call this with an untilted heading
 */
void
ahrs_compass_update(
	real_t			heading
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
}


static void
euler2quat( void )
{
	const real_t		phi     = ahrs_euler[0] / 2.0;
	const real_t		theta   = ahrs_euler[1] / 2.0;
	const real_t		psi     = ahrs_euler[2] / 2.0;

	const real_t		shphi0   = sin( phi );
	const real_t		chphi0   = cos( phi );

	const real_t		shtheta0 = sin( theta );
	const real_t		chtheta0 = cos( theta );

	const real_t		shpsi0   = sin( psi );
	const real_t		chpsi0   = cos( psi );

	q0 =  chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0;
	q1 = -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0;
	q2 =  chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0;
	q3 =  chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0;
}


/*
 * Initialize the AHRS state data and covariance matrix.  The user
 * should have filled in the pqr and accel vectors before calling this.
 * They should also have set ahrs_euler[2] from an untilted compass reading
 * and the ahrs_euler[0], ahrs_euler[1] from the accel2roll / accel2pitch values.
 */
void
_ahrs_init(
	const int16_t *		mag
)
{
	index_t			i;

	euler2quat();
	compute_DCM();
	
	ahrs_euler[2] = untilt_compass( mag );

	euler2quat();
	compute_DCM();

	/* Initialize the bias terms so the filter doesn't work as hard */
	bias[0] = pqr[0];
	bias[1] = pqr[1];
	bias[2] = pqr[2];

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

/*
 * Check for a valid reading from the compass.  If there is not one yet,
 * start a reading.  If the last one failed, emit the error and try again.
 */
#ifdef PNI_MAG
void
pni_poll( void )
{
	if( pni_valid < 0 )
	{
		puts( "PNIINV\r\n" );
		pni_valid = 0;
		return;
	}

	if( pni_valid == 0 )
	{
		pni_read_axis();
		return;
	}

	/* Valid reading! */
	pni_valid = 0;
}
#endif //PNI_MAG

//some inline functions
static inline real_t
accel2roll( void )
{
	return -atan2( accel[1], -accel[2] );
}


static inline real_t
accel2pitch()
{
	index_t         i;
	uint16_t        g2 = 0;

	/* Compute the square of the magnitude of the acceleration */
	for( i=0 ; i<3 ; i++ )
		g2 += accel[i] * accel[i];

	return -asin( accel[0] / -sqrt( g2 ) );
}




/** - Here start the Paparazzi englued code 
 *  - TODO : move this code to another file will be good
 */
uint8_t ahrs_state = AHRS_NOT_INITIALIZED;

static inline void
accel_init( void )
{
	//accel : paparazzi mean accel buffer init
	adc_buf_channel(IMU_ADC_ACCELX, &buf_accelX, DEFAULT_AV_NB_SAMPLE);
	adc_buf_channel(IMU_ADC_ACCELY, &buf_accelY, DEFAULT_AV_NB_SAMPLE);
	adc_buf_channel(IMU_ADC_ACCELZ, &buf_accelZ, DEFAULT_AV_NB_SAMPLE);

}

static inline void
gyro_init( void)
{
    	//gyro : paparazzi initialization
	//nothing todo
}

static inline void
imu_calibration( uint8_t reset )
{
	static uint32_t dec = IMU_INIT_EULER_DOT_NB_SAMPLES_MIN; 
	static int16_t pqr_min[3],pqr_max[3];
	int8_t i;

	//assuming that pqr sample is new is new....
	
	//init-algo
	if (reset)
	    dec = IMU_INIT_EULER_DOT_NB_SAMPLES_MIN;

	if (dec == IMU_INIT_EULER_DOT_NB_SAMPLES_MIN)
	{
		for(i=0;i<3;i++)
			pqr_min[i] = pqr_max[i] = pqr[i];//<--from_fbw.euler_dot[i];
		return;
	}									
	if (dec-- > 0)
	{
		//Saving min and max
		for(i=0;i<3;i++)
		{
			//pqr[i] = from_fbw.euler_dot[i];//already done
			pqr_min[i] = (pqr_min[i] < pqr[i])? pqr_min[i] : pqr[i];
			pqr_max[i] = (pqr_max[i] > pqr[i])? pqr_max[i] : pqr[i];
		}

		//testing variance
		for(i=0;i<3;i++) 
		{ 
			if ((pqr_max[i]- pqr_min[i]) > IMU_INIT_EULER_DOT_VARIANCE_MAX)
			{       
				//re-init algo
				dec = IMU_INIT_EULER_DOT_NB_SAMPLES_MIN;
				return;
			}
		}
			
	}
	else
	{
	    	//entering in the end of calibration ;-)
	
	    	//we take the middle for pqr
		//pqr is already normalized and scaled by the fbw mcu
		for(i=0;i<3;i++)
			pqr[i] = (pqr_min[i] + pqr_max[i])/2;

		/** - Time of Accel Now
		 *
		 */

		const int16_t ax_m	= buf_accelX.sum/buf_accelX.av_nb_sample;
		const int16_t ay_m	= buf_accelY.sum/buf_accelY.av_nb_sample;
		const int16_t az_m	= buf_accelZ.sum/buf_accelZ.av_nb_sample;
	
		//Geeting ax ay az from this adc buffer mean
		accel[0] = IMU_ADC_ACCELX_SIGN (ax_m - IMU_ADC_ACCELX_ZERO);
		accel[1] = IMU_ADC_ACCELY_SIGN (ay_m - IMU_ADC_ACCELY_ZERO);
		accel[2] = IMU_ADC_ACCELZ_SIGN (az_m - IMU_ADC_ACCELZ_ZERO);

		//Enf of ahrs_init ;-)
#ifdef PNI_MAG
		pni_poll();		
		_ahrs_init( pni_values );
#else
		_ahrs_init( NULL );
#endif //PNI_MAG
		ahrs_state = AHRS_RUNNING;
	}
	
}


static inline void
pqr_update( void )
{
    //do nothing here in paparazzi
    //pqr is filled "asymchronously" by the exported function ahrs_save_pqr_from_fbw
}



void
roll_update( void )
{
	//have we got new datas to eat, tha the question ???????????
    	const int16_t	ay_m	= buf_accelY.sum/buf_accelY.av_nb_sample;
	const int16_t	az_m	= buf_accelZ.sum/buf_accelZ.av_nb_sample;
	
	//Geeting ax ay az from this adc buffer mean
	// accel[0] is not needed for roll_update.
	accel[1] = IMU_ADC_ACCELY_SIGN (ay_m - IMU_ADC_ACCELY_ZERO);
	accel[2] = IMU_ADC_ACCELZ_SIGN (az_m - IMU_ADC_ACCELZ_ZERO);

	ahrs_euler[0] =  accel2roll();

}


static inline void
pitch_update( void )
{
	//have we got new datas to eat, tha the question ???????????
    	const int16_t	ax_m	= buf_accelX.sum/buf_accelX.av_nb_sample;
	const int16_t	ay_m	= buf_accelY.sum/buf_accelY.av_nb_sample;
	const int16_t	az_m	= buf_accelZ.sum/buf_accelZ.av_nb_sample;
	
	//Geeting ax ay az from this adc buffer mean
	accel[0] = IMU_ADC_ACCELX_SIGN (ax_m - IMU_ADC_ACCELX_ZERO);
	accel[1] = IMU_ADC_ACCELY_SIGN (ay_m - IMU_ADC_ACCELY_ZERO);
	accel[2] = IMU_ADC_ACCELZ_SIGN (az_m - IMU_ADC_ACCELZ_ZERO);

	ahrs_euler[1] = accel2pitch();
}



static inline void
compass_update( void )
{
#ifdef PNI_MAG
    	index_t			i;

	/* Swap the sensor readings to rotate front to back */
	pni_values[0] = -pni_values[0];
	pni_values[1] = -pni_values[1];
/*
	putc( 'M' );
	for( i=0 ; i<3 ; i++ )
		put_int16_t( pni_values[i] );
	putnl();
*/
	ahrs_euler[2] = untilt_compass( pni_values );
#else
	//faking the Compass
	ahrs_euler[2] = 0;
#endif //PNI_MAG
}

#define reset()	((void(*)(void))0)()





//Exported Function to paparazzi



void ahrs_save_pqr_from_fbw( void )
{
	//we take the gyro data from the spi data
	//No transformation is needed for pqr
	//FBW not scale the data for the moment so we transform here
	pqr[0] = from_fbw.euler_dot[0] /** IMU_ADC_ROLL_DOT_SIGN IMU_ADC_ROLL_DOT_SCALE*/;
	pqr[1] = from_fbw.euler_dot[1] /** IMU_ADC_PITCH_DOT_SIGN IMU_ADC_PITCH_DOT_SCALE*/;
	pqr[2] = from_fbw.euler_dot[2] /** IMU_ADC_YAW_DOT_SIGN IMU_ADC_YAW_DOT_SCALE*/;
}


//ahrs_init have to be call in the mainllop init part
//ahrs have to be call during the mainllop

void ahrs_init(uint8_t do_calibration)
{
	if (ahrs_state == AHRS_IMU_CALIBRATION)
		return;
	
    	if(ahrs_state == AHRS_NOT_INITIALIZED)
	{
#ifdef PNI_MAG
		pni_init();
#endif //PNI_MAG
		accel_init();
		gyro_init();
	}

	if(do_calibration)
	{
		ahrs_state = AHRS_IMU_CALIBRATION;
	    	imu_calibration(TRUE);
	    	//end of ahrs_init is done in imu_calibration when calib is done
	}
	else
	{
	    	roll_update();
		pitch_update();
	    
	    	//ahrs_euler[0] = accel2roll();
		//ahrs_euler[1] = accel2pitch();

		//Enf of ahrs_init ;-)
#ifdef PNI_MAG
		pni_poll();		
		_ahrs_init( pni_values );
#else
		_ahrs_init( NULL );
#endif //PNI_MAG
		ahrs_state = AHRS_RUNNING;
	}
		

}

void ahrs_update()
{
	static uint8_t	step = 0;
	
	if (ahrs_state != AHRS_RUNNING)
	{
		if (ahrs_state == AHRS_IMU_CALIBRATION)
			imu_calibration(FALSE);
		return;
	}
	
    	if( isnan( q0 ) )
	{
		//puts( "\r\nFilter NaN! Reset!\r\n" );
		reset();
	}
		
	pqr_update();
	ahrs_state_update();

#ifdef PNI_MAG
	pni_poll();
#endif //PNI_MAG
	if( step == 0 )
	{
		roll_update();
		ahrs_roll_update( ahrs_euler[0] );
		step = 1;
	} else
	if( step == 1 )
	{
		pitch_update();
		ahrs_pitch_update( ahrs_euler[1] );
		step = 2;
	} else
	if( step == 2 )
	{
#ifdef PNI_MAG
	    	//Updating Compass
	    	compass_update();
#else
		//Fucking Compass
		ahrs_euler[2] = 0;
#endif //PNI_MAG
		ahrs_compass_update( ahrs_euler[2] );
		step = 0;
	}
}

#endif //SECTION_IMU_ANALOG

