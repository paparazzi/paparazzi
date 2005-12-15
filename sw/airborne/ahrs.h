/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id$
 *
 * Fast AHRS object almost ready for use on microcontrollers
 * Fixed-point mode can be use !
 *
 * (c) 2003 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2005 Jean-Pierre Dumont <jpxDOTdumontATwanadooDOTfr>
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

/** \file ahrs.h
 *  \brief Attitude Heading Reference System (gyros, accels and magneto
 * filtered through a Kalman)
 *
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

/** @name ahrs_state possible values */
//@{ 
#define AHRS_NOT_INITIALIZED		0
#define AHRS_IMU_CALIBRATION		1
#define AHRS_RUNNING			2
//@} 

/** Current internal state */
extern uint8_t ahrs_state;


/** Restarts the ahrs with zero_calibration phase */
void ahrs_init( uint8_t do_zero_calibration );

/** Udates state with accels and compas (if available) */
void ahrs_update( void );

#if (!defined IMU_GYROS_CONNECTED_TO_AP) || (IMU_GYROS_CONNECTED_TO_AP == 0)

/** Function to be called when gyro data are available (through the link to
    the fbw mcu */
void ahrs_gyro_update( void );
#endif //IMU_GYROS_CONNECTED_TO_AP

#endif //__AHRS_
