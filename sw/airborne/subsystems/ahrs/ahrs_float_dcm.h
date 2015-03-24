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

/**
 * @file subsystems/ahrs/ahrs_float_dcm.h
 *
 * Attitude estimation for fixedwings based on the DCM.
 *
 * Theory: http://code.google.com/p/gentlenav/downloads/list  file DCMDraft2.pdf
 *
 */

#ifndef AHRS_FLOAT_DCM_H
#define AHRS_FLOAT_DCM_H

#include <inttypes.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"
#include "subsystems/gps.h"

enum AhrsDCMStatus {
  AHRS_DCM_UNINIT,
  AHRS_DCM_RUNNING
};

struct AhrsFloatDCM {
  struct FloatRates gyro_bias;
  struct FloatRates rate_correction;

  struct FloatEulers ltp_to_imu_euler;
  struct FloatRates imu_rate;

  float gps_speed;
  float gps_acceleration;
  float gps_course;
  bool_t gps_course_valid;
  uint8_t gps_age;

  struct OrientationReps body_to_imu;

  enum AhrsDCMStatus status;
  bool_t is_aligned;
};
extern struct AhrsFloatDCM ahrs_dcm;

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

extern void ahrs_dcm_init(void);
extern void ahrs_dcm_set_body_to_imu(struct OrientationReps *body_to_imu);
extern void ahrs_dcm_set_body_to_imu_quat(struct FloatQuat *q_b2i);
extern bool_t ahrs_dcm_align(struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                             struct Int32Vect3 *lp_mag);
extern void ahrs_dcm_propagate(struct Int32Rates *gyro, float dt);
extern void ahrs_dcm_update_accel(struct Int32Vect3 *accel);
extern void ahrs_dcm_update_mag(struct Int32Vect3 *mag);
extern void ahrs_dcm_update_gps(struct GpsState *gps_s);

#endif // AHRS_FLOAT_DCM_H
