/*
 * $Id: analogimu.c $
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

/** \file analogimu.c
 *  \brief Analog IMU Routines
 *
 */

#include "generated/airframe.h"

#if ! (defined SITL || defined HITL)


// Actual Inertial Measurements
#include "subsystems/imu/imu_analog.h"

// AHRS attitude computations
#include "dcm.h"
#include "estimator.h"
#include "analogimu.h"

// Debugging and output
#include "led.h"
#include "mcu_periph/uart.h"
//#include "downlink.h"
//#include "ap_downlink.h"
#include "sys_time.h"
#include "math/pprz_algebra_int.h"


#endif

// remotely settable
float imu_roll_neutral = RadOfDeg(IMU_ROLL_NEUTRAL_DEFAULT);
float imu_pitch_neutral = RadOfDeg(IMU_PITCH_NEUTRAL_DEFAULT);

#if ! (defined SITL || defined HITL)

void analog_imu_offset_set( void ) {
  // read IMU, really?
  imu_periodic();

  imu.gyro_neutral.p = analog_imu_values[0];
  imu.gyro_neutral.q = analog_imu_values[1];
  imu.gyro_neutral.r = analog_imu_values[2];

  // do not set accel neutrals on startup, use the neutrals from the airfame file
  //imu.accel_neutral.x = analog_imu_values[3];
  //imu.accel_neutral.y = analog_imu_values[4];
  //imu.accel_neutral.z  = analog_imu_values[5];

  // Z channel should read
  // FIXME uugh...
  //imu.accel_neutral.z +=  (9.81f / IMU_ACCEL_Z_SENS);
}

// functions

void analog_imu_downlink( void ) {
  //uint8_t id = 0;
  //float time = GET_CUR_TIME_FLOAT();
  //time *= 1000;//secs to msecs
  //int mx = 0;
  //int my = 0;
  //int mz = 0;
  //DOWNLINK_SEND_HB_FILTER( DefaultChannel,&time, &accel[ACC_X],&accel[ACC_Y],&accel[ACC_Z],&gyro[G_ROLL],&gyro[G_PITCH],&gyro[G_YAW],&heading,&mx,&my,&mz,&euler[EULER_ROLL],&euler[EULER_PITCH],&euler[EULER_YAW], &imu_roll_neutral, &imu_pitch_neutral );
}


void estimator_update_state_analog_imu( void ) {

  /* Offset is set dynamic on Ground*/
  /*Gyro_Vector[0]= -gyro_to_zero[G_ROLL]   + gyro[G_ROLL];
  Gyro_Vector[1]= -gyro_to_zero[G_PITCH]  + gyro[G_PITCH];
  Gyro_Vector[2]= -gyro_to_zero[G_PITCH]  + gyro[G_YAW];*/

  //FIXME run aligner to set gyro neutrals on ground

  /* converts gyro to floating point */
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro);
  Gyro_Vector[0]= gyro_float.p;
  Gyro_Vector[1]= gyro_float.q;
  Gyro_Vector[2]= gyro_float.r;

  struct FloatVect3 accel_float;
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);
  Accel_Vector[0] = accel_float.x;
  Accel_Vector[1] = accel_float.y;
  Accel_Vector[2] = accel_float.z;


  Matrix_update();
  Normalize();


  Drift_correction();
  Euler_angles();

  // return euler angles to phi and theta
  estimator_phi = euler.phi-imu_roll_neutral;
  //estimator_phi = angle[ANG_ROLL];
  estimator_theta = euler.theta-imu_pitch_neutral;
  //estimator_theta = angle[ANG_PITCH];
  estimator_psi = euler.psi;

}
#endif
