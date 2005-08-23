/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id$
 *
 * Fast AHRS object almost ready for use on microcontrollers
 *
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 */
#ifndef _fast_ahrs_h_
#define _fast_ahrs_h_

#include <inttypes.h>


typedef float		real_t;
typedef uint8_t		index_t;


/*
 * We have seven variables in our state -- the quaternion attitude
 * estimate and three gyro bias values.  The state time update equation
 * comes from the IMU gyro sensor readings:
 *
 *	Q_dot		= Wxq(pqr) * Q
 *	Bias_dot	= 0
 */
extern real_t	X[7];
extern real_t	quat[4];
extern real_t	q0;
extern real_t	q1;
extern real_t	q2;
extern real_t	q3;
extern real_t	bias[3];
extern real_t	bias_p;
extern real_t	bias_q;
extern real_t	bias_r;

extern real_t	pqr[3];
extern int16_t	accel[3];


/*
 * The euler estimate will be updated less frequently than the
 * quaternion, but some applications are ok with that.
 */
extern real_t	euler[3];


//exported functions
extern void ahrs_init( void );
extern void ahrs_update( void );

#endif
