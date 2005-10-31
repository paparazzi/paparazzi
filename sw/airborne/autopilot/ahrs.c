/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id$
 *
 * Fast AHRS object almost ready for use on microcontrollers
 *
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2005 Jean-Pierre Dumont <flyingtuxATfreeDOTfr>
 */

//#define TIME_OPTIM

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
#include "uart.h"

#ifdef PNI_MAG
#include "pni.h"
#endif //PNI_MAG

#ifdef SECTION_IMU_ANALOG


#define C_ONE		1.0
#define C_TWO		2.0
#define C_PI		((real_t) 3.14159265358979323846264338327950)
#define C_HALF_PI	(C_PI/2)
#define C_TWO_PI	(C_PI*2)


/*
 * dt is our time step.  It is important to be close to the actual
 * frequency with which ahrs_state_update() is called with the body
 * angular rates.
 */

#define	dt		1/15 //0,066666667  //1/15
#define CONFIG_SPLIT_COVARIANCE
#ifdef CONFIG_SPLIT_COVARIANCE
#define	dt_covariance	2 * dt
#else
#define dt_covariance	dt
#endif //CONFIG_SLIT_COVARIANCE

#define AHRS_DEBUG

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
real_t pqr[3];//rad/s
real_t accel[3];//in G

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

//jpdumont adds... (take care on asm ahrs.S)

/*raw datas
 */
int16_t gyro[3];//direct mean from gyro 
int16_t gyro_zero[3];//this datas will by re-initialiazed during startup

int16_t accel_raw[3];//direct lean from adxl 
int16_t accel_raw_zero[3];

/* real_t datas
 */

real_t gyro_scale[3];
real_t accel_scale[3];

/*
 * paparazzi adc_buff
 */
static struct adc_buf buf_accelX;
static struct adc_buf buf_accelY;
static struct adc_buf buf_accelZ;

/*
 * only if gyro are connected on ap
 */
#if (defined IMU_GYROS_CONNECTED_TO_AP) && (IMU_GYROS_CONNECTED_TO_AP != 0)
static struct adc_buf buf_gyroP;
static struct adc_buf buf_gyroQ;
static struct adc_buf buf_gyroR;
#endif //IMU_GYROS_CONNECTED_TO_AP

#ifdef AHRS_DEBUG
char		float_buf[12];
void
put_float(
	const float 		f
)
{
	uint8_t			i;


	dtostrf(
		f,
		sizeof( float_buf )-1,
		5,
		float_buf
	);

	for( i=0 ; i<sizeof(float_buf) ; i++ )
	{
		char c = float_buf[i];
		if( !c )
			break;
		uart0_transmit( c );

		/* Stop on a NaN */
		if( i == 2 && c == 'N' )
		{
			uart0_transmit( ' ' );
			break;
		}
	}
}
#endif //AHRS_DEBUG


/*
 * Simple helper to normalize our quaternion attitude estimate.  We could
 * probably do this at a lower time step to save on calls to sqrt().
 */
void
norm_quat( void )
{
//	index_t		i;
	real_t		mag2 = 0;
	real_t		mag = 0;


	/*for( i=0 ; i<4 ; i++ )
		mag2 += quat[i] * quat[i];*/
	mag2 += quat[0] * quat[0];
	mag2 += quat[1] * quat[1];
	mag2 += quat[2] * quat[2];
	mag2 += quat[3] * quat[3];
	mag = sqrt( mag2 );


#ifdef AHRS_DEBUG
	if (mag == 0)
	{
		uart0_print_string("\nnorm_quat:div by 0 !\n");
		mag = mag2;
	}
#endif //AHRS_DEBUG

	/*for( i=0 ; i<4 ; i++ )
		quat[i] /= mag;*/
	quat[0] /= mag;
	quat[1] /= mag;
	quat[2] /= mag;
	quat[3] /= mag;
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
	/*for( i=4 ; i<7 ; i++ )
		Pdot[i][i] += Q_gyro;*/
	Pdot[4][4] += Q_gyro;
	Pdot[5][5] += Q_gyro;
	Pdot[6][6] += Q_gyro;
	
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
				else if( i == k )
					Pdot[i][j] += P[i][k] * A[j][k];
				else
					Pdot[i][j] += A[i][k] * P[k][j]
						   +  P[i][k] * A[j][k];
			}
	/* Compute P = P + Pdot * dt */
	/*for( i=0 ; i<7 ; i++ )
		for( j=0 ; j<7 ; j++ )
			P[i][j] += Pdot[i][j] * dt_covariance;*/
	P[0][0] += Pdot[0][0] * dt_covariance;
	P[0][1] += Pdot[0][1] * dt_covariance;
	P[0][2] += Pdot[0][2] * dt_covariance;
	P[0][3] += Pdot[0][3] * dt_covariance;
	P[0][4] += Pdot[0][4] * dt_covariance;
	P[0][5] += Pdot[0][5] * dt_covariance;
	P[0][6] += Pdot[0][6] * dt_covariance;
	
	P[1][0] += Pdot[1][0] * dt_covariance;
	P[1][1] += Pdot[1][1] * dt_covariance;
	P[1][2] += Pdot[1][2] * dt_covariance;
	P[1][3] += Pdot[1][3] * dt_covariance;
	P[1][4] += Pdot[1][4] * dt_covariance;
	P[1][5] += Pdot[1][5] * dt_covariance;
	P[1][6] += Pdot[1][6] * dt_covariance;

	P[2][0] += Pdot[2][0] * dt_covariance;
	P[2][1] += Pdot[2][1] * dt_covariance;
	P[2][2] += Pdot[2][2] * dt_covariance;
	P[2][3] += Pdot[2][3] * dt_covariance;
	P[2][4] += Pdot[2][4] * dt_covariance;
	P[2][5] += Pdot[2][5] * dt_covariance;
	P[2][6] += Pdot[2][6] * dt_covariance;

	P[3][0] += Pdot[3][0] * dt_covariance;
	P[3][1] += Pdot[3][1] * dt_covariance;
	P[3][2] += Pdot[3][2] * dt_covariance;
	P[3][3] += Pdot[3][3] * dt_covariance;
	P[3][4] += Pdot[3][4] * dt_covariance;
	P[3][5] += Pdot[3][5] * dt_covariance;
	P[3][6] += Pdot[3][6] * dt_covariance;

	P[4][0] += Pdot[4][0] * dt_covariance;
	P[4][1] += Pdot[4][1] * dt_covariance;
	P[4][2] += Pdot[4][2] * dt_covariance;
	P[4][3] += Pdot[4][3] * dt_covariance;
	P[4][4] += Pdot[4][4] * dt_covariance;
	P[4][5] += Pdot[4][5] * dt_covariance;
	P[4][6] += Pdot[4][6] * dt_covariance;

	P[5][0] += Pdot[5][0] * dt_covariance;
	P[5][1] += Pdot[5][1] * dt_covariance;
	P[5][2] += Pdot[5][2] * dt_covariance;
	P[5][3] += Pdot[5][3] * dt_covariance;
	P[5][4] += Pdot[5][4] * dt_covariance;
	P[5][5] += Pdot[5][5] * dt_covariance;
	P[5][6] += Pdot[5][6] * dt_covariance;

	P[6][0] += Pdot[6][0] * dt_covariance;
	P[6][1] += Pdot[6][1] * dt_covariance;
	P[6][2] += Pdot[6][2] * dt_covariance;
	P[6][3] += Pdot[6][3] * dt_covariance;
	P[6][4] += Pdot[6][4] * dt_covariance;
	P[6][5] += Pdot[6][5] * dt_covariance;
	P[6][6] += Pdot[6][6] * dt_covariance;
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
//	index_t			i;
//	index_t			j;

	compute_A_quat();
	memset( Qdot, 0, sizeof(Qdot) );

	/* Compute Q_dot = Wxq(pqr) * Q (storing temp in C) */	
	/*for( i=0 ; i<4 ; i++ )
	{
		for( j=0 ; j<4 ; j++ )
		{
			if( i == j )
				continue;
			Qdot[i] += A[i][j] * quat[j];
		}
	}*/

	//Qdot[0] += A[0][0] * quat[0];
	Qdot[0] += A[0][1] * quat[1];
	Qdot[0] += A[0][2] * quat[2];
	Qdot[0] += A[0][3] * quat[3];
	
	Qdot[1] += A[1][0] * quat[0];
	//Qdot[1] += A[1][1] * quat[1];
	Qdot[1] += A[1][2] * quat[2];
	Qdot[1] += A[1][3] * quat[3];
	
	Qdot[2] += A[2][0] * quat[0];
	Qdot[2] += A[2][1] * quat[1];
	//Qdot[2] += A[2][2] * quat[2];
	Qdot[2] += A[2][3] * quat[3];
	
	Qdot[3] += A[3][0] * quat[0];
	Qdot[3] += A[3][1] * quat[1];
	Qdot[3] += A[3][2] * quat[2];
	//Qdot[3] += A[3][3] * quat[3];

/* Compute Q = Q + Q_dot * dt */
	/*for( i=0 ; i<4 ; i++ )
		quat[i] += Qdot[i] * dt;*/
	quat[0] += Qdot[0] * dt;
	quat[1] += Qdot[1] * dt;
	quat[2] += Qdot[2] * dt;
	quat[3] += Qdot[3] * dt;
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
	}
	else 
	{
		covariance_state = 0;
		covariance_update_1();
		return;
	}
#else
	covariance_update_0();
	covariance_update_1();
#endif //CONFIG_SPLIT_COVARIANCE
}




/*
 * Compute the five elements of the DCM that we use for our
 * rotations and Jacobians.  This is used by several other functions
 * to speedup their computations.
 */
static void
compute_DCM( void )
{
	const real_t q2xq2 = q2*q2; 
    dcm00 = C_ONE - 2*(q2xq2 + q3*q3);
	dcm01 =     	2*(q1*q2 + q0*q3);
	dcm02 =     	2*(q1*q3 - q0*q2);
	dcm12 =     	2*(q2*q3 + q0*q1);
	dcm22 = C_ONE - 2*(q1*q1 + q2xq2);
}


/*
 * Compute the Jacobian of the measurements to the system states.
 * You must have already computed the DCM for the quaternion before
 * calling this function.
 */
static inline void
compute_dphi_dq( void )
{
//	index_t			i;
	const real_t tmp = dcm22*dcm22 + dcm12*dcm12;

#ifdef AHRS_DEBUG
	if (tmp == 0)
	{
		uart0_print_string("\ncompute_dphi_dq:div by 0 !\n");
		return;
	}
#endif //AHRS_DEBUG

	const real_t phi_err =  C_TWO / tmp;

	C[0] = q1 * dcm22;
	C[1] = q0 * dcm22 + 2 * q1 * dcm12;
	C[2] = q3 * dcm22 + 2 * q2 * dcm12;
	C[3] = q2 * dcm22;

	/*for( i=0 ; i<4 ; i++ )
		C[i] *= phi_err;*/
	C[0] *= phi_err;
	C[1] *= phi_err;
	C[2] *= phi_err;
	C[3] *= phi_err;
}

static inline void
compute_dtheta_dq( void )
{
	real_t tmp = C_ONE - dcm02*dcm02;
	
#ifdef AHRS_DEBUG
	if (tmp == 0)
	{
		uart0_print_string("\ncompute_dtheta_dq:1:div by 0 !\n");
		tmp = real_t_min;
	}
	if (tmp < 0)
	{
		uart0_print_string("\ncompute_dtheta_dq:1:sqrt of negativ value !\n");
		tmp = real_t_min; 
	}
#endif //AHRS_DEBUG

	real_t sqrt_tmp = sqrt(tmp);

#ifdef AHRS_DEBUG
	if (sqrt_tmp == 0)
	{
		uart0_print_string("\ncompute_dtheta_dq:2:div by 0 !\n");
		sqrt_tmp = tmp;
	}
#endif //AHRS_DEBUG

	const real_t theta_err	= -C_TWO / sqrt_tmp;

	C[0] = -q2 * theta_err;
	C[1] =  q3 * theta_err;
	C[2] = -q0 * theta_err;
	C[3] =  q1 * theta_err;
}

static inline void
compute_dpsi_dq( void )
{
//	index_t			i;
	const real_t tmp = dcm00*dcm00 + dcm01*dcm01;

#ifdef AHRS_DEBUG
	if (tmp == 0)
	{
		uart0_print_string("\ncompute_dpsi_dq::div by 0 !\n");
		return;
	}
#endif //AHRS_DEBUG

	const real_t psi_err	=  C_TWO / tmp;

	C[0] = (q3 * dcm00);
	C[1] = (q2 * dcm00);
	C[2] = (q1 * dcm00 + 2 * q2 * dcm01);
	C[3] = (q0 * dcm00 + 2 * q3 * dcm01);

	/*for( i=0 ; i<4 ; i++ )
		C[i] *= psi_err;*/
	C[0] *= psi_err;
	C[1] *= psi_err;
	C[2] *= psi_err;
	C[3] *= psi_err;
}



/*
 * Translate our quaternion attitude estimate into Euler angles.
 * This is expensive, so don't do it often.  You must have already
 * computed the DCM for the quaternion before calling.
 */
static inline void
compute_euler_roll( void )
{
#ifdef AHRS_DEBUG
	if (dcm22 == 0)
	{
		uart0_print_string("\ncompute_euler_roll:atan2(Y,0) !\n");
		return;
	}
#endif //AHRS_DEBUG
	ahrs_euler[0] = atan2( dcm12, dcm22 );
}

static inline void
compute_euler_pitch( void )
{
	//TODO : perhaps we should verify that dcm02 is in [-1;1]
	ahrs_euler[1] = -asin( dcm02 );
}

static inline void
compute_euler_heading( void )
{
#ifdef AHRS_DEBUG
	if (dcm00 == 0)
	{
		uart0_print_string("\ncompute_euler_heading:atan2(Y,0) !\n");
		return;
	}
#endif //AHRS_DEBUG
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
//	index_t		i;
//	index_t		j;
//	index_t		k;

	memset( (void*) PCt, 0, sizeof( PCt ) );

	/* Compute PCt = P * C_tranpose */
	/*for( i=0 ; i<7 ; i++ )
		for( k=0 ; k<4 ; k++ )
			PCt[i] += P[i][k] * C[k];*/
	PCt[0] += P[0][0] * C[0];
	PCt[0] += P[0][1] * C[1];
	PCt[0] += P[0][2] * C[2];
	PCt[0] += P[0][3] * C[3];
		
	PCt[1] += P[1][0] * C[0];
	PCt[1] += P[1][1] * C[1];
	PCt[1] += P[1][2] * C[2];
	PCt[1] += P[1][3] * C[3];
		
	PCt[2] += P[2][0] * C[0];
	PCt[2] += P[2][1] * C[1];
	PCt[2] += P[2][2] * C[2];
	PCt[2] += P[2][3] * C[3];
		
	PCt[3] += P[3][0] * C[0];
	PCt[3] += P[3][1] * C[1];
	PCt[3] += P[3][2] * C[2];
	PCt[3] += P[3][3] * C[3];
	
	PCt[4] += P[4][0] * C[0];
	PCt[4] += P[4][1] * C[1];
	PCt[4] += P[4][2] * C[2];
	PCt[4] += P[4][3] * C[3];
	
	PCt[5] += P[5][0] * C[0];
	PCt[5] += P[5][1] * C[1];
	PCt[5] += P[5][2] * C[2];
	PCt[5] += P[5][3] * C[3];
	
	PCt[6] += P[6][0] * C[0];
	PCt[6] += P[6][1] * C[1];
	PCt[6] += P[6][2] * C[2];
	PCt[6] += P[6][3] * C[3];

	/* Compute E = C * PCt + R */
	E = R_axis;
	/*for( i=0 ; i<4 ; i++ )
		E += C[i] * PCt[i];*/
	E += C[0] * PCt[0];
	E += C[1] * PCt[1];
	E += C[2] * PCt[2];
	E += C[3] * PCt[3];

	/* Compute the inverse of E */
#ifdef AHRS_DEBUG
	if (E == 0)
	{
		uart0_print_string("\nrun_kalman:div by 0 !\n");
		return;
	}
#endif //AHRS_DEBUG
	
	E = C_ONE / E;

	/* Compute K = P * C_tranpose * inv(E) */
	/*for( i=0 ; i<7 ; i++ )
		K[i] = PCt[i] * E;*/
	K[0] = PCt[0] * E;
	K[1] = PCt[1] * E;
	K[2] = PCt[2] * E;
	K[3] = PCt[3] * E;
	K[4] = PCt[4] * E;
	K[5] = PCt[5] * E;
	K[6] = PCt[6] * E;

	/* Update our covariance matrix: P = P - K * C * P */

	/* Compute CP = C * P, reusing the PCt array. */
	memset( (void*) PCt, 0, sizeof( PCt ) );
	/*for( j=0 ; j<7 ; j++ )
		for( k=0 ; k<4 ; k++ )
		    	PCt[j] += C[k] * P[k][j];*/
	PCt[0] += C[0] * P[0][0];
	PCt[0] += C[1] * P[1][0];
	PCt[0] += C[2] * P[2][0];
	PCt[0] += C[3] * P[3][0];
	
	PCt[1] += C[0] * P[0][1];
	PCt[1] += C[1] * P[1][1];
	PCt[1] += C[2] * P[2][1];
	PCt[1] += C[3] * P[3][1];
	
	PCt[2] += C[0] * P[0][2];
	PCt[2] += C[1] * P[1][2];
	PCt[2] += C[2] * P[2][2];
	PCt[2] += C[3] * P[3][2];
	
	PCt[3] += C[0] * P[0][3];
	PCt[3] += C[1] * P[1][3];
	PCt[3] += C[2] * P[2][3];
	PCt[3] += C[3] * P[3][3];
	
	PCt[4] += C[0] * P[0][4];
	PCt[4] += C[1] * P[1][4];
	PCt[4] += C[2] * P[2][4];
	PCt[4] += C[3] * P[3][4];
	
	PCt[5] += C[0] * P[0][5];
	PCt[5] += C[1] * P[1][5];
	PCt[5] += C[2] * P[2][5];
	PCt[5] += C[3] * P[3][5];
	
	PCt[6] += C[0] * P[0][6];
	PCt[6] += C[1] * P[1][6];
	PCt[6] += C[2] * P[2][6];
	PCt[6] += C[3] * P[3][6];

	/* Compute P -= K * CP (aliased to PCt) */
	/*for( i=0 ; i<7 ; i++ )
		for( j=0 ; j<7 ; j++ )			
			P[i][j] -= K[i] * PCt[j];*/
	P[0][0] -= K[0] * PCt[0];
	P[0][1] -= K[0] * PCt[1];
	P[0][2] -= K[0] * PCt[2];
	P[0][3] -= K[0] * PCt[3];
	P[0][4] -= K[0] * PCt[4];
	P[0][5] -= K[0] * PCt[5];
	P[0][6] -= K[0] * PCt[6];
		
	P[1][0] -= K[1] * PCt[0];
	P[1][1] -= K[1] * PCt[1];
	P[1][2] -= K[1] * PCt[2];
	P[1][3] -= K[1] * PCt[3];
	P[1][4] -= K[1] * PCt[4];
	P[1][5] -= K[1] * PCt[5];
	P[1][6] -= K[1] * PCt[6];
		
	P[2][0] -= K[2] * PCt[0];
	P[2][1] -= K[2] * PCt[1];
	P[2][2] -= K[2] * PCt[2];
	P[2][3] -= K[2] * PCt[3];
	P[2][4] -= K[2] * PCt[4];
	P[2][5] -= K[2] * PCt[5];
	P[2][6] -= K[2] * PCt[6];
	
	P[3][0] -= K[3] * PCt[0];
	P[3][1] -= K[3] * PCt[1];
	P[3][2] -= K[3] * PCt[2];
	P[3][3] -= K[3] * PCt[3];
	P[3][4] -= K[3] * PCt[4];
	P[3][5] -= K[3] * PCt[5];
	P[3][6] -= K[3] * PCt[6];
	
	P[4][0] -= K[4] * PCt[0];
	P[4][1] -= K[4] * PCt[1];
	P[4][2] -= K[4] * PCt[2];
	P[4][3] -= K[4] * PCt[3];
	P[4][4] -= K[4] * PCt[4];
	P[4][5] -= K[4] * PCt[5];
	P[4][6] -= K[4] * PCt[6];
	
	P[5][0] -= K[5] * PCt[0];
	P[5][1] -= K[5] * PCt[1];
	P[5][2] -= K[5] * PCt[2];
	P[5][3] -= K[5] * PCt[3];
	P[5][4] -= K[5] * PCt[4];
	P[5][5] -= K[5] * PCt[5];
	P[5][6] -= K[5] * PCt[6];
	
	P[6][0] -= K[6] * PCt[0];
	P[6][1] -= K[6] * PCt[1];
	P[6][2] -= K[6] * PCt[2];
	P[6][3] -= K[6] * PCt[3];
	P[6][4] -= K[6] * PCt[4];
	P[6][5] -= K[6] * PCt[5];
	P[6][6] -= K[6] * PCt[6];

	/* Update our state: X += K * error */
	/*for( i=0 ; i<7 ; i++ )
		X[i] += K[i] * error;*/
	X[0] += K[0] * error;
	X[1] += K[1] * error;
	X[2] += K[2] * error;
	X[3] += K[3] * error;
	X[4] += K[4] * error;
	X[5] += K[5] * error;
	X[6] += K[6] * error;
	
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
		roll += C_TWO_PI;
	if( roll > C_PI )
		roll -= C_TWO_PI;

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
	if( pitch < -C_HALF_PI )
		pitch += C_PI;
	if( pitch > C_HALF_PI )
		pitch -= C_PI;

	run_kalman( R_pitch, pitch );

	// We'll normalize our quaternion here only
	norm_quat();
}

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
		heading += C_TWO_PI;
	if( heading > C_PI )
		heading -= C_TWO_PI;

	run_kalman( R_yaw, heading );
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

#ifdef AHRS_DEBUG
	if (ctheta == 0)
	{
		uart0_print_string("\nuntilt_compass:div by 0 !\n");
		return 0 ;//prhaps should we return other thing
	}
#endif //AHRS_DEBUG
	
	const real_t	mn = ctheta * mag[0]
		- (dcm12 * mag[1] + dcm22 * mag[2]) * dcm02 / ctheta;

	const real_t	me =
		(dcm22 * mag[1] - dcm12 * mag[2]) / ctheta;
#ifdef AHRS_DEBUG
	if (mn == 0)
	{
		uart0_print_string("\nuntilt_compass:atan2(Y,0) !\n");
		return -((me > 0) ? C_HALF_PI : C_HALF_PI);
	}
#endif //AHRS_DEBUG
	return -atan2( me, -mn );
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

//END OF USING_FP WORK

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
//	index_t			i;

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
	/*for( i=0 ; i<4 ; i++ )
		P[i][i] = C_ONE;*/
	P[0][0] = C_ONE;
	P[1][1] = C_ONE;
	P[2][2] = C_ONE;
	P[3][3] = C_ONE;
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
#ifdef AHRS_DEBUG
	if (accel[2] == 0)
	{
		uart0_print_string("\naccel2roll:atan2(Y,0) !\n");
		return -((accel[1]>0) ? C_HALF_PI : -C_HALF_PI);
	}
#endif //AHRS_DEBUG
	return -atan2( accel[1], -accel[2] );
}


static inline real_t
accel2pitch()
{
//	index_t         i;
	real_t          g2 = 0;

	/* Compute the square of the magnitude of the acceleration */

	/*for( i=0 ; i<3 ; i++ )
		g2 += accel[i] * accel[i];*/
	g2 += accel[0] * accel[0];
	g2 += accel[1] * accel[1];
	g2 += accel[2] * accel[2];

	real_t tmp = -sqrt(g2);

#ifdef AHRS_DEBUG
	if (tmp == 0)
	{
		uart0_print_string("\naccel2pitch:div by 0 !\n");
		tmp = -g2;
	}
#endif //AHRS_DEBUG
	
	return -asin( accel[0] / tmp );
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

	//Default accel_raw_zeo initialisation
	accel_raw_zero[0] = IMU_ADC_ACCELX_ZERO;
	accel_raw_zero[1] = IMU_ADC_ACCELY_ZERO;
	accel_raw_zero[2] = IMU_ADC_ACCELZ_ZERO;

	//default accel_scale initialisation
	accel_scale[0] = IMU_ADC_ACCELX_SCALE;
	accel_scale[1] = IMU_ADC_ACCELY_SCALE;
	accel_scale[2] = IMU_ADC_ACCELZ_SCALE;
}

static inline void
gyro_init( void)
{
    //gyro : paparazzi initialization
	//nothing todo
#if (defined IMU_GYROS_CONNECTED_TO_AP) && (IMU_GYROS_CONNECTED_TO_AP != 0)
	//gyro : paparazzi mean gyro buffer init(only if gyro are connected to ap)
	adc_buf_channel(IMU_ADC_ROLL_DOT, &buf_gyroP, DEFAULT_AV_NB_SAMPLE);
	adc_buf_channel(IMU_ADC_PITCH_DOT, &buf_gyroQ, DEFAULT_AV_NB_SAMPLE);
	adc_buf_channel(IMU_ADC_YAW_DOT, &buf_gyroR, DEFAULT_AV_NB_SAMPLE);
#endif //IMU_GYROS_CONNECTED_TO_AP
	
	//Default gyro_zero initialisation
	gyro_zero[0] = IMU_ADC_ROLL_DOT_ZERO;
	gyro_zero[1] = IMU_ADC_PITCH_DOT_ZERO;
	gyro_zero[2] = IMU_ADC_YAW_DOT_ZERO;
	
	//Default gyro_scale Initialisation
	gyro_scale[0] = IMU_ADC_ROLL_DOT_SCALE;
	gyro_scale[1] = IMU_ADC_PITCH_DOT_SCALE;
	gyro_scale[2] = IMU_ADC_YAW_DOT_SCALE;
}


static inline void
pqr_update( void )
{
#if (defined IMU_GYROS_CONNECTED_TO_AP) && (IMU_GYROS_CONNECTED_TO_AP != 0)
	ahrs_gyro_update();
	
	pqr[0] = IMU_ADC_ROLL_DOT_SIGN  ((real_t)(gyro[0] - gyro_zero[0])) * gyro_scale[0];
    	pqr[1] = IMU_ADC_PITCH_DOT_SIGN ((real_t)(gyro[1] - gyro_zero[1])) * gyro_scale[1];
    	pqr[2] = IMU_ADC_YAW_DOT_SIGN   ((real_t)(gyro[2] - gyro_zero[2])) * gyro_scale[2];
#else
    //ahrs_gyro_update is called from the mainloop.c when fbw gyro comming if gyro are connected to fbw
    //data are signed or not and unoffseted so ?
    //TODO: do it as you feel
    	pqr[0] = /*IMU_ADC_ROLL_DOT_SIGN*/  ((real_t)(gyro[0] /*- gyro_zero[0]*/)) * gyro_scale[0];
    	pqr[1] = /*IMU_ADC_PITCH_DOT_SIGN*/ ((real_t)(gyro[1] /*- gyro_zero[1]*/)) * gyro_scale[1];
    	pqr[2] = /*IMU_ADC_YAW_DOT_SIGN*/   ((real_t)(gyro[2] /*- gyro_zero[2]*/)) * gyro_scale[2];
#endif // IMU_GYROS_CONNECTED_TO_AP
}


void
roll_update( void )
{
	//Geeting ax ay az from this adc buffer mean
    accel_raw[1] = buf_accelY.sum/buf_accelY.av_nb_sample;
	accel_raw[2] = buf_accelZ.sum/buf_accelZ.av_nb_sample;
	
	//accel[0] is not needed for roll_update.
	accel[1] = IMU_ADC_ACCELY_SIGN ((real_t)(accel_raw[1] - accel_raw_zero[1])) * accel_scale[1];
	accel[2] = IMU_ADC_ACCELZ_SIGN ((real_t)(accel_raw[2] - accel_raw_zero[2])) * accel_scale[2];

	ahrs_euler[0] =  accel2roll();

}


static inline void
pitch_update( void )
{
	//Geeting ax ay az from this adc buffer mean
    accel_raw[0] = buf_accelX.sum/buf_accelX.av_nb_sample;
	accel_raw[1] = buf_accelY.sum/buf_accelY.av_nb_sample;
	accel_raw[2] = buf_accelZ.sum/buf_accelZ.av_nb_sample;
	
	accel[0] = IMU_ADC_ACCELX_SIGN ((real_t)(accel_raw[0] - accel_raw_zero[0])) * accel_scale[0];
	accel[1] = IMU_ADC_ACCELY_SIGN ((real_t)(accel_raw[1] - accel_raw_zero[1])) * accel_scale[1];
	accel[2] = IMU_ADC_ACCELZ_SIGN ((real_t)(accel_raw[2] - accel_raw_zero[2])) * accel_scale[2];

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
	ahrs_euler[2] = untilt_compass( pni_values );
#else
	//faking the Compass
	ahrs_euler[2] = 0;
#endif //PNI_MAG
}

#define reset()	((void(*)(void))0)()


//Exported Function to paparazzi

static inline void
zero_calibration( uint8_t reset )
{
	static int16_t dec = IMU_INIT_EULER_DOT_NB_SAMPLES_MIN; 
	static int16_t gyro_min[3],gyro_max[3];
	uint8_t i;

#ifdef AHRS_DEBUG		
	//init-algo
	uart0_print_string(" dec = ");
 	uart0_print_hex16(dec); 
	uart0_transmit('\n');
#endif //AHRS_DEBUG
	if (reset)
	    dec = IMU_INIT_EULER_DOT_NB_SAMPLES_MIN;

#if (defined IMU_GYROS_CONNECTED_TO_AP) && (IMU_GYROS_CONNECTED_TO_AP != 0)
	ahrs_gyro_update();//only if gyros are connected to the ap
#endif //IMU_GYROS_CONNECTED_TO_AP

	if (dec == IMU_INIT_EULER_DOT_NB_SAMPLES_MIN)
	{
		for(i=0;i<3;i++)
			gyro_min[i] = gyro_max[i] = gyro[i];//<--from_fbw.euler_dot[i];
		dec--;
		return;
	}
	
	if (dec)
	{
		//Saving min and max
		for(i=0;i<3;i++)
		{
			//gyro[i] = from_fbw.euler_dot[i];//already done
			gyro_min[i] = (gyro_min[i] < gyro[i])? gyro_min[i] : gyro[i];
			gyro_max[i] = (gyro_max[i] > gyro[i])? gyro_max[i] : gyro[i];
		}

		//testing variance
		for(i=0;i<3;i++) 
		{ 
			if ((gyro_max[i]- gyro_min[i]) > IMU_INIT_EULER_DOT_VARIANCE_MAX)
			{       
				//re-init algo
				dec = IMU_INIT_EULER_DOT_NB_SAMPLES_MIN;
				return;
			}
		}
		dec--;	
	}
	else
	{
	    	//entering in the end of calibration ;-)
	
	    	//we take the middle for pqr
		//normally pqr should be normalized and scaled by the fbw mcu
		//but offset is determined here and with kalman
	 	for(i=0;i<3;i++)
			gyro_zero[i] = (gyro_min[i] + gyro_max[i])/2;

#ifdef AHRS_DEBUG
		uart0_print_string("gyro_zero : ");
    		uart0_print_hex16(gyro_zero[0]);
    		uart0_transmit(',');
    		uart0_print_hex16(gyro_zero[1]);
    		uart0_transmit(',');
    		uart0_print_hex16(gyro_zero[2]);
		uart0_transmit('\n');
#endif //AHRS_DEBUG

		//fixe here accel_raw_zero
		accel_raw_zero[0] = buf_accelX.sum/buf_accelX.av_nb_sample;
		accel_raw_zero[1] = buf_accelY.sum/buf_accelY.av_nb_sample;
		//accel_raw_zero[2] = buf_accelZ.sum/buf_accelZ.av_nb_sample + IMU_ADC_ACCELZ_RAW_RANGE/2;

#ifdef AHRS_DEBUG		
		uart0_print_string("accel_raw_zero : ");
    		uart0_print_hex16(accel_raw_zero[0]);
    		uart0_transmit(',');
    		uart0_print_hex16(accel_raw_zero[1]);
    		uart0_transmit(',');
    		uart0_print_hex16(accel_raw_zero[2]);
		uart0_transmit('\n');
#endif //AHRS_DEBUG

		pqr_update();
		roll_update();
		pitch_update();
		
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


//AHRS EXPORTED FUNCTION
void ahrs_init(uint8_t do_zero_calibration)
{
	//fp_test();return;
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

	if(do_zero_calibration)
	{
		ahrs_state = AHRS_IMU_CALIBRATION;
	    	zero_calibration(TRUE);
	    	//end of ahrs_init is done in zero_calibration when calib is done
	}
	else
	{  
	    	pqr_update();
	        roll_update();
		pitch_update();

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

void ahrs_gyro_update( void )
{
#if (defined IMU_GYROS_CONNECTED_TO_AP) && (IMU_GYROS_CONNECTED_TO_AP != 0)
	gyro[0] = buf_gyroP.sum/buf_gyroP.av_nb_sample;
	gyro[1] = buf_gyroQ.sum/buf_gyroQ.av_nb_sample;
	gyro[2] = buf_gyroR.sum/buf_gyroR.av_nb_sample;
#else
	//taking data from fbw
	gyro[0] = from_fbw.euler_dot[0];
	gyro[1] = from_fbw.euler_dot[1];
	gyro[2] = from_fbw.euler_dot[2];
#endif //IMU_GYROS_CONNECTED_TO_AP
}

void ahrs_update()
{
    static uint8_t	step = 0;

#ifdef AHRS_DEBUG	
	/*if (ahrs_state == AHRS_RUNNING)
 		uart0_transmit('R');
	else if (ahrs_state == AHRS_IMU_CALIBRATION)
		uart0_transmit('C');
	else if (ahrs_state == AHRS_NOT_INITIALIZED)
		uart0_transmit('N');*/

	uint16_t t1 = TCNT2;//timer_now();
#endif //AHRS_DEBUG
	
	
	if (ahrs_state != AHRS_RUNNING)
	{
		if (ahrs_state == AHRS_IMU_CALIBRATION)
			zero_calibration(FALSE);
		return;
	}
	
	if( isnan( q0 ) )
	{
		uart0_print_string( "\nFilter NaN! Reset!\n" );
		//reset();
	}
	switch(step)
	{
		case 0:
			//this one takes les than 6.2 ms
			pqr_update();
			ahrs_state_update();
			break;
		case 1:
			//this one takes les than 8.6 ms (atan2)
			roll_update();
			ahrs_roll_update( ahrs_euler[0] );
			break;
		case 2:
			//this one takes les than 6.4 ms (asin)
			pitch_update();
			ahrs_pitch_update( ahrs_euler[1] );
			break;
		case 3:
			//this one takes les than 6.9 ms 
#ifdef PNI_MAG
			pni_poll();//perhaps should I call this more often
	    	//Updating Compass
	    	compass_update();
#else
			//Fucking Compass
			ahrs_euler[2] = 0;
#endif //PNI_MAG
			ahrs_compass_update( ahrs_euler[2] );
			break;
	}
    step = (step<3) ? step+1 : 0;

#ifdef AHRS_DEBUG
	uint16_t t2 = TCNT2;//timer_now();
	uint16_t t3 = t2 > t1 ? t2 - t1 : t1 - t2;
	float tms= t3;
	tms *= 0.064f;
	switch(step)
	{
		case 0:
			uart0_print_string("\nEuler = ");
			put_float(ahrs_euler[0]);
			put_float(ahrs_euler[1]);
			put_float(ahrs_euler[2]);
			uart0_print_string("in");
			put_float(tms);
			break;
		case 1:
		case 2:
			uart0_print_string("  +");
			put_float(tms);
			break;
		case 3:
			put_float(tms);
			uart0_print_string("  ms");
	}	
#endif //AHRS_DEBUG

}

#endif //SECTION_IMU_ANALOG
