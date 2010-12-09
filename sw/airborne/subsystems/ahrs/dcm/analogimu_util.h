/*
 * $Id: analogimu_imu.h $
 *
 * Copyright (C) 2010 Christoph Niemann
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
 *
 */

/** \file analogimu_util.h
 *  \brief Analog IMU Utilities
 *
 */

#ifndef _ANALOGIMU_UTIL_H_
#define _ANALOGIMU_UTIL_H_

#include "std.h"

// defines
/** defines for gyro[] indicies */
enum gyro_idx_t { G_ROLL, G_PITCH, G_YAW, G_LAST };
/** defines for accel[] indicies */
enum accel_idx_t { ACC_X, ACC_Y, ACC_Z, ACC_LAST };
/** defines for stick[] indicies */
enum stick_idx_t { STICK_ROLL, STICK_PITCH, STICK_YAW, STICK_THRUST,
		   STICK_SWR, STICK_SWL, STICK_CH6, STICK_CH7, STICK_LAST };
/** defines for angle[] indicies */
enum angle_idx_t { ANG_ROLL, ANG_PITCH, ANG_YAW, ANG_LAST };
/** defines for gyro_to_zero[] indicies */
enum gyro_to_zero_idx_t { GO_ROLL, GO_PITCH, GO_YAW, GO_LAST };

// variables
extern volatile float gyro[];
/** acceleration in ms2 */
extern volatile float accel[];
/** angle in rad */
extern volatile float angle[];
/** magnet heading \todo heading ? */
extern volatile float heading;
/** analog_raw[] analog downlink arry */
extern uint16_t analog_raw[];

extern volatile float gyro_to_zero[];

extern bool_t analog_imu_reset( void );

void analogimu_delay( void );


#endif
