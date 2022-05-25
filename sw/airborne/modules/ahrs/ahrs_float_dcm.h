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
 * @file modules/ahrs/ahrs_float_dcm.h
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
#include "modules/gps/gps.h"

enum AhrsDCMStatus {
  AHRS_DCM_UNINIT,
  AHRS_DCM_RUNNING
};

struct AhrsFloatDCM {
  struct FloatRates gyro_bias;
  struct FloatRates rate_correction;

  struct FloatEulers ltp_to_body_euler;
  struct FloatRates body_rate;

  float gps_speed;
  float gps_acceleration;
  float gps_course;
  bool gps_course_valid;
  uint8_t gps_age;

  enum AhrsDCMStatus status;
  bool is_aligned;
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

#ifndef AHRS_FLOAT_MIN_SPEED_GPS_COURSE
#define AHRS_FLOAT_MIN_SPEED_GPS_COURSE 5 // m/s
#endif

#define PERFORMANCE_REPORTING 1
#if PERFORMANCE_REPORTING == 1
extern int renorm_sqrt_count;
extern int renorm_blowup_count;
extern float imu_health;
#endif

extern void ahrs_dcm_init(void);
extern bool ahrs_dcm_align(struct FloatRates *lp_gyro, struct FloatVect3 *lp_accel,
                             struct FloatVect3 *lp_mag);
extern void ahrs_dcm_propagate(struct FloatRates *gyro, float dt);
extern void ahrs_dcm_update_accel(struct FloatVect3 *accel);
extern void ahrs_dcm_update_mag(struct FloatVect3 *mag);
extern void ahrs_dcm_update_gps(struct GpsState *gps_s);

#endif // AHRS_FLOAT_DCM_H
