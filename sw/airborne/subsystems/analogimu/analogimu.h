/*
 * $Id: analogimu.h $
 *  
 * Copyright (C) 2010 Oliver Riesener, Christoph Niemann
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

/** \file analogimu.h
 *  \brief Analog IMU Interface
 *
 */

#ifndef _ANALOGIMU_H_
#define _ANALOGIMU_H_
// types
#define uint16_t short int

#include "std.h"
#include "generated/airframe.h"

extern float imu_roll_neutral;
extern float imu_pitch_neutral;


// ADC0 Slots
#define ADC_YAW 1
#define ADC_PITCH 2
#define ADC_ROLL 3
#define ADC_OFF 4

// ADC1 Slots
#define ADC_ACCX (3+8)
#define ADC_ACCY (4+8)
#define ADC_ACCZ (5+8)
#define ADC_VBAT (6+8)
#define ADC_ALT  (7+8)

//functions
void analog_imu_init( void );
void analog_imu_update( void ); 
void analog_imu_downlink( void );
void analogconversion( void );
void matrix_transpose(float *c, float *a, short m, short n);
void accel2euler( void );
void accel2ms2( void );
void estimator_update_state_analog_imu( void );
void gyro2rads( void );
void analog_imu_offset_set( void );

#endif // _ANALOGIMU_H_
