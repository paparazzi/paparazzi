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
#include "sys_time.h"
#include "math/pprz_algebra_int.h"


#endif

// remotely settable
float imu_roll_neutral = RadOfDeg(IMU_ROLL_NEUTRAL_DEFAULT);
float imu_pitch_neutral = RadOfDeg(IMU_PITCH_NEUTRAL_DEFAULT);

#if ! (defined SITL || defined HITL)

void estimator_update_state_analog_imu( void ) {

  //FIXME run aligner to set gyro neutrals on ground

  /* converts gyro to floating point */
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro);
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);

  Matrix_update();
  Normalize();

  // FIXME convert gps data here

  Drift_correction();
  Euler_angles();

  // export results to estimator
  estimator_phi = euler.phi-imu_roll_neutral;
  estimator_theta = euler.theta-imu_pitch_neutral;
  estimator_psi = euler.psi;

}
#endif
