/*
 * Copyright (C) 2010 The Paparazzi Team
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

/** \file ahrs_float_dcm.h
 *  \brief Attitude estimation for fixedwings based on the DCM
 *
 */

#ifndef AHRS_FLOAT_DCM_H
#define AHRS_FLOAT_DCM_H

#include <inttypes.h>
#include "math/pprz_algebra_float.h"

struct AhrsFloatDCM {
  struct FloatRates gyro_bias;
  struct FloatRates rate_correction;
  /*
    Holds float version of IMU alignement
    in order to be able to run against the fixed point
    version of the IMU
  */
  struct FloatQuat body_to_imu_quat;
  struct FloatRMat body_to_imu_rmat;
};
extern struct AhrsFloatDCM ahrs_impl;

extern float imu_roll_neutral;
extern float imu_pitch_neutral;

void ahrs_update_fw_estimator(void);

// DCM Parameters

//#define Kp_ROLLPITCH 0.2
#define Kp_ROLLPITCH 0.015
#define Ki_ROLLPITCH 0.000010
#define Kp_YAW 1.2          //High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005

#define GRAVITY 9.81


#define OUTPUTMODE 1
// Mode 0 = DCM integration without Ki gyro bias
// Mode 1 = DCM integration with Kp and Ki
// Mode 2 = direct accelerometer -> euler

#define MAGNETOMETER 1
extern float MAG_Heading;

#define PERFORMANCE_REPORTING 0
#if PERFORMANCE_REPORTING == 1
extern int renorm_sqrt_count;
extern int renorm_blowup_count;
extern float imu_health;
#endif

#endif // AHRS_FLOAT_DCM_H
