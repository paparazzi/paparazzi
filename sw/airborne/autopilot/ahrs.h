/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id$
 *
 * Fast AHRS object almost ready for use on microcontrollers
 * Fixed-point mode can be use !
 *
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2005 Jean-Pierre Dumont <jpxDOTdumontATwanadooDOTfr>
 */
#ifndef __AHRS_
#define __AHRS_

#include <inttypes.h>

typedef float		real_t;
#define			real_t_min	0.000000001
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
extern real_t	accel[3];

extern int16_t accel_raw[3];
extern int16_t gyro_raw[3];
extern int16_t accel_raw_zero[3];
extern int16_t gyro_raw_zero[3];

/*
 * The euler estimate will be updated less frequently than the
 * quaternion, but some applications are ok with that.
 */
extern real_t	ahrs_euler[3];

//haere are the ahrs_state possible values
#define AHRS_NOT_INITIALIZED		0
#define AHRS_IMU_CALIBRATION		1
#define AHRS_RUNNING			2


//export internal state
extern uint8_t ahrs_state;


//exported mainloop functions
extern void ahrs_init( uint8_t do_zero_calibration );//will restart the ahrs with zero_calibration phase
//export function
extern void ahrs_gyro_update( void );//needed by paparazzi achitecture
extern void ahrs_update( void );//will update state with accels and compas if we have

#endif //__AHRS_
