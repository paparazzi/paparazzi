/*
 * Released under Creative Commons License
 *
 * 2010 The Paparazzi Team
 *
 *
 * Based on Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson (Wired) and Nathan Sindle (SparkFun).
 * Version 1.0 for flat board updated by Doug Weibel and Jose Julio
 *
 * Modified at Hochschule Bremen, Germany
 * 2010 Heinrich Warmers, Christoph Niemann, Oliver Riesener
 *
 */

/** \file ahrs_float_dcm.h
 *  \brief Attitude estimation for fixedwings based on the DCM
 *  Theory: http://code.google.com/p/gentlenav/downloads/list  file DCMDraft2.pdf
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

extern float ins_roll_neutral;
extern float ins_pitch_neutral;

void ahrs_update_fw_estimator(void);

// DCM Parameters

//#define Kp_ROLLPITCH 0.2
#define Kp_ROLLPITCH 0.015
#define Ki_ROLLPITCH 0.000010
#define Kp_YAW 0.9          //High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005

#define GRAVITY 9.81


#ifndef OUTPUTMODE
#define OUTPUTMODE 1
#endif
// Mode 0 = DCM integration without Ki gyro bias
// Mode 1 = DCM integration with Kp and Ki
// Mode 2 = direct accelerometer -> euler


#define PERFORMANCE_REPORTING 1
#if PERFORMANCE_REPORTING == 1
extern int renorm_sqrt_count;
extern int renorm_blowup_count;
extern float imu_health;
#endif

#endif // AHRS_FLOAT_DCM_H
