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
 * @file subsystems/ahrs/ahrs_float_dcm.c
 *
 * Attitude estimation for fixedwings based on the DCM.
 *
 * Theory: http://code.google.com/p/gentlenav/downloads/list  file DCMDraft2.pdf
 *
 * Options:
 *  - USE_MAGNETOMETER_ONGROUND: use magnetic compensation before takeoff only while GPS course not good
 *  - USE_AHRS_GPS_ACCELERATIONS: forward acceleration compensation from GPS speed
 *
 */

#include "std.h"

#include "subsystems/ahrs/ahrs_float_dcm.h"
#include "subsystems/ahrs/ahrs_float_utils.h"
#include "autopilot.h"  // launch detection

#include "subsystems/ahrs/ahrs_float_dcm_algebra.h"
#include "math/pprz_algebra_float.h"

#if USE_GPS
#include "subsystems/gps.h"
#endif

#include <string.h>

#include "led.h"

#if FLOAT_DCM_SEND_DEBUG
// FIXME Debugging Only
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#endif

struct AhrsFloatDCM ahrs_dcm;

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

struct FloatVect3 accel_float = {0, 0, 0};
struct FloatVect3 mag_float = {0, 0, 0};

float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

float DCM_Matrix[3][3]       = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3]    = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

#if USE_MAGNETOMETER
float MAG_Heading_X = 1;
float MAG_Heading_Y = 0;
#endif

static void compute_ahrs_representations(void);
static inline void set_dcm_matrix_from_rmat(struct FloatRMat *rmat);

void Normalize(void);
void Drift_correction(void);
void Matrix_update(float dt);

#if PERFORMANCE_REPORTING == 1
int renorm_sqrt_count = 0;
int renorm_blowup_count = 0;
float imu_health = 0.;
#endif


static inline void set_dcm_matrix_from_rmat(struct FloatRMat *rmat)
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      DCM_Matrix[i][j] = RMAT_ELMT(*rmat, j, i);
    }
  }
}

void ahrs_dcm_init(void)
{
  ahrs_dcm.status = AHRS_DCM_UNINIT;
  ahrs_dcm.is_aligned = false;

  /* init ltp_to_imu euler with zero */
  FLOAT_EULERS_ZERO(ahrs_dcm.ltp_to_imu_euler);

  FLOAT_RATES_ZERO(ahrs_dcm.imu_rate);

  /* set inital filter dcm */
  set_dcm_matrix_from_rmat(orientationGetRMat_f(&ahrs_dcm.body_to_imu));

  ahrs_dcm.gps_speed = 0;
  ahrs_dcm.gps_acceleration = 0;
  ahrs_dcm.gps_course = 0;
  ahrs_dcm.gps_course_valid = false;
  ahrs_dcm.gps_age = 100;
}

bool ahrs_dcm_align(struct FloatRates *lp_gyro, struct FloatVect3 *lp_accel,
                      struct FloatVect3 *lp_mag)
{
  /* Compute an initial orientation using euler angles */
  ahrs_float_get_euler_from_accel_mag(&ahrs_dcm.ltp_to_imu_euler, lp_accel, lp_mag);

  /* Convert initial orientation in quaternion and rotation matrice representations. */
  struct FloatRMat ltp_to_imu_rmat;
  float_rmat_of_eulers(&ltp_to_imu_rmat, &ahrs_dcm.ltp_to_imu_euler);

  /* set filter dcm */
  set_dcm_matrix_from_rmat(&ltp_to_imu_rmat);

  /* use averaged gyro as initial value for bias */
  ahrs_dcm.gyro_bias = *lp_gyro;

  ahrs_dcm.status = AHRS_DCM_RUNNING;
  ahrs_dcm.is_aligned = true;

  return true;
}


void ahrs_dcm_propagate(struct FloatRates *gyro, float dt)
{
  /* unbias rate measurement */
  RATES_DIFF(ahrs_dcm.imu_rate, *gyro, ahrs_dcm.gyro_bias);

  /* Uncouple Motions */
#ifdef IMU_GYRO_P_Q
  float dp = 0, dq = 0, dr = 0;
  dp += ahrs_dcm.imu_rate.q * IMU_GYRO_P_Q;
  dp += ahrs_dcm.imu_rate.r * IMU_GYRO_P_R;
  dq += ahrs_dcm.imu_rate.p * IMU_GYRO_Q_P;
  dq += ahrs_dcm.imu_rate.r * IMU_GYRO_Q_R;
  dr += ahrs_dcm.imu_rate.p * IMU_GYRO_R_P;
  dr += ahrs_dcm.imu_rate.q * IMU_GYRO_R_Q;

  ahrs_dcm.imu_rate.p += dp;
  ahrs_dcm.imu_rate.q += dq;
  ahrs_dcm.imu_rate.r += dr;
#endif

  Matrix_update(dt);

  Normalize();

  compute_ahrs_representations();
}

void ahrs_dcm_update_gps(struct GpsState *gps_s)
{
  static float last_gps_speed_3d = 0;

#if USE_GPS
  if (gps_s->fix >= GPS_FIX_3D) {
    ahrs_dcm.gps_age = 0;
    ahrs_dcm.gps_speed = gps_s->speed_3d / 100.;

    if (gps_s->gspeed >= 500) { //got a 3d fix and ground speed is more than 5.0 m/s
      ahrs_dcm.gps_course = ((float)gps_s->course) / 1.e7;
      ahrs_dcm.gps_course_valid = true;
    } else {
      ahrs_dcm.gps_course_valid = false;
    }
  } else {
    ahrs_dcm.gps_age = 100;
  }
#endif

  ahrs_dcm.gps_acceleration += (((ahrs_dcm.gps_speed - last_gps_speed_3d) * 4.0f)  - ahrs_dcm.gps_acceleration) / 5.0f;
  last_gps_speed_3d = ahrs_dcm.gps_speed;
}


void ahrs_dcm_update_accel(struct FloatVect3 *accel)
{
  // DCM filter uses g-force as positive
  // accelerometer measures [0 0 -g] in a static case
  accel_float.x = -accel->x;
  accel_float.y = -accel->y;
  accel_float.z = -accel->z;


  ahrs_dcm.gps_age ++;
  if (ahrs_dcm.gps_age < 50) {    //Remove centrifugal acceleration and longitudinal acceleration
#if USE_AHRS_GPS_ACCELERATIONS
    PRINT_CONFIG_MSG("AHRS_FLOAT_DCM uses GPS acceleration.")
    accel_float.x += ahrs_dcm.gps_acceleration;      // Longitudinal acceleration
#endif
    accel_float.y += ahrs_dcm.gps_speed * Omega[2];  // Centrifugal force on Acc_y = GPS_speed*GyroZ
    accel_float.z -= ahrs_dcm.gps_speed * Omega[1];  // Centrifugal force on Acc_z = GPS_speed*GyroY
  } else {
    ahrs_dcm.gps_speed = 0;
    ahrs_dcm.gps_acceleration = 0;
    ahrs_dcm.gps_age = 100;
  }

  Drift_correction();
}


void ahrs_dcm_update_mag(struct FloatVect3 *mag)
{
#if USE_MAGNETOMETER
MESSAGE("MAGNETOMETER FEEDBACK NOT TESTED YET")

  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cosf(ahrs_dcm.ltp_to_imu_euler.phi);
  sin_roll = sinf(ahrs_dcm.ltp_to_imu_euler.phi);
  cos_pitch = cosf(ahrs_dcm.ltp_to_imu_euler.theta);
  sin_pitch = sinf(ahrs_dcm.ltp_to_imu_euler.theta);


  // Pitch&Roll Compensation:
  MAG_Heading_X = mag->x * cos_pitch + mag->y * sin_roll * sin_pitch + mag->z * cos_roll * sin_pitch;
  MAG_Heading_Y = mag->y * cos_roll - mag->z * sin_roll;

  /*
   *
    // Magnetic Heading
    Heading = atan2(-Head_Y,Head_X);

    // Declination correction (if supplied)
    if( declination != 0.0 )
    {
        Heading = Heading + declination;
        if (Heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
            Heading -= (2.0 * M_PI);
        else if (Heading < -M_PI)
            Heading += (2.0 * M_PI);
    }

    // Optimization for external DCM use. Calculate normalized components
    Heading_X = cos(Heading);
    Heading_Y = sin(Heading);
  */

#if FLOAT_DCM_SEND_DEBUG
  struct FloatVect3 ltp_mag;

  ltp_mag.x = MAG_Heading_X;
  ltp_mag.y = MAG_Heading_Y;

  // Downlink
  RunOnceEvery(10, DOWNLINK_SEND_IMU_MAG(DefaultChannel, DefaultDevice, &ltp_mag.x, &ltp_mag.y, &ltp_mag.z));
#endif

  // Magnetic Heading
  // MAG_Heading = atan2(mag->y, -mag->x);

#else // !USE_MAGNETOMETER
  // get rid of unused param warning...
  mag = mag;
#endif
}

void Normalize(void)
{
  float error = 0;
  float temporary[3][3];
  float renorm = 0;
  uint8_t problem = false;

  // Find the non-orthogonality of X wrt Y
  error = -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; //eq.19

  // Add half the XY error to X, and half to Y
  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error);           //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error);           //eq.19
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);  //eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);  //eq.19

  // The third axis is simply set perpendicular to the first 2. (there is not correction of XY based on Z)
  Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

  // Normalize lenght of X
  renorm = Vector_Dot_Product(&temporary[0][0], &temporary[0][0]);
  // a) if norm is close to 1, use the fast 1st element from the tailer expansion of SQRT
  // b) if the norm is further from 1, use a real sqrt
  // c) norm is huge: disaster! reset! mayday!
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm = .5 * (3 - renorm);                                       //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm = 1. / sqrtf(renorm);
#if PERFORMANCE_REPORTING == 1
    renorm_sqrt_count++;
#endif
  } else {
    problem = true;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
  }
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

  // Normalize lenght of Y
  renorm = Vector_Dot_Product(&temporary[1][0], &temporary[1][0]);
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm = .5 * (3 - renorm);                                              //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm = 1. / sqrtf(renorm);
#if PERFORMANCE_REPORTING == 1
    renorm_sqrt_count++;
#endif
  } else {
    problem = true;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
  }
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

  // Normalize lenght of Z
  renorm = Vector_Dot_Product(&temporary[2][0], &temporary[2][0]);
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm = .5 * (3 - renorm);                                              //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm = 1. / sqrtf(renorm);
#if PERFORMANCE_REPORTING == 1
    renorm_sqrt_count++;
#endif
  } else {
    problem = true;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
  }
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);

  // Reset on trouble
  if (problem) {                // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
    set_dcm_matrix_from_rmat(orientationGetRMat_f(&ahrs_dcm.body_to_imu));
    problem = false;
  }
}

// strong structural vibrations can prevent to perform the drift correction
// so accel magnitude is filtered before computing the weighting heuristic
#ifndef ACCEL_WEIGHT_FILTER
#define ACCEL_WEIGHT_FILTER 8
#endif

// the weigthing is function of the length of a band of 1G by default
// so <0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0
// adjust the band size if needed, the value should be >0
#ifndef ACCEL_WEIGHT_BAND
#define ACCEL_WEIGHT_BAND 1.f
#endif

void Drift_correction()
{
  //Compensation the Roll, Pitch and Yaw drift.
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  static float Accel_filtered = 0.f;
  float Accel_magnitude;
  float Accel_weight;
  float Integrator_magnitude;

  // Local Working Variables
  float errorRollPitch[3];
  float errorYaw[3];
  float errorCourse;

  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrtf(accel_float.x * accel_float.x + accel_float.y * accel_float.y + accel_float.z * accel_float.z);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
#if ACCEL_WEIGHT_FILTER
  Accel_filtered = (Accel_magnitude + (ACCEL_WEIGHT_FILTER - 1) * Accel_filtered) / ACCEL_WEIGHT_FILTER;
#else // set ACCEL_WEIGHT_FILTER to 0 to disable filter
  Accel_filtered = Accel_magnitude;
#endif
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info according to band size (min value is 0.1 to prevent division by zero)
  Accel_weight = Clip(1.f - (2.f / Max(0.1f,ACCEL_WEIGHT_BAND)) * fabsf(1.f - Accel_filtered), 0.f, 1.f);


#if PERFORMANCE_REPORTING == 1
  {
    //amount added was determined to give imu_health a time constant about twice the time constant of the roll/pitch drift correction
    float tempfloat = ((Accel_weight - 0.5) *  256.0f);
    imu_health += tempfloat;
    Bound(imu_health, 129, 65405);
  }
#endif

  Vector_Cross_Product(&errorRollPitch[0], &accel_float.x, &DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);

  Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
  Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);

  //*****YAW***************

#if USE_MAGNETOMETER
  // We make the gyro YAW drift correction based on compass magnetic heading
//  float mag_heading_x = cos(MAG_Heading);
//  float mag_heading_y = sin(MAG_Heading);
  // 2D dot product
  //Calculating YAW error
  errorCourse = (DCM_Matrix[0][0] * MAG_Heading_Y) + (DCM_Matrix[1][0] * MAG_Heading_X);
  //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse);

  Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW);
  Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding  Proportional.

  Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW);
  Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I

#else // Use GPS Ground course to correct yaw gyro drift

  if (ahrs_dcm.gps_course_valid) {
    float course = ahrs_dcm.gps_course - M_PI; //This is the runaway direction of you "plane" in rad
    float COGX = cosf(course); //Course overground X axis
    float COGY = sinf(course); //Course overground Y axis

    errorCourse = (DCM_Matrix[0][0] * COGY) - (DCM_Matrix[1][0] * COGX); //Calculating YAW error
    //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse);

    Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW);
    Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW);
    Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I
  }
#if USE_MAGNETOMETER_ONGROUND
  PRINT_CONFIG_MSG("AHRS_FLOAT_DCM uses magnetometer prior to takeoff and GPS during flight")
  else if (autopilot.launch == FALSE) {
    float COGX = mag->x; // Non-Tilt-Compensated (for filter stability reasons)
    float COGY = mag->y; // Non-Tilt-Compensated (for filter stability reasons)

    errorCourse = (DCM_Matrix[0][0] * COGY) - (DCM_Matrix[1][0] * COGX); //Calculating YAW error
    //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse);

    // P only
    Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW / 10.0);
    Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding  Proportional.fi
  }
#endif // USE_MAGNETOMETER_ONGROUND
#endif

  //  Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
  Integrator_magnitude = sqrtf(Vector_Dot_Product(Omega_I, Omega_I));
  if (Integrator_magnitude > RadOfDeg(300)) {
    Vector_Scale(Omega_I, Omega_I, 0.5f * RadOfDeg(300) / Integrator_magnitude);
  }


}
/**************************************************/

void Matrix_update(float dt)
{
  Vector_Add(&Omega[0], &ahrs_dcm.imu_rate.p, &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

#if OUTPUTMODE==1    // With corrected data (drift correction)
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -dt * Omega_Vector[2]; //-z
  Update_Matrix[0][2] = dt * Omega_Vector[1]; //y
  Update_Matrix[1][0] = dt * Omega_Vector[2]; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -dt * Omega_Vector[0]; //-x
  Update_Matrix[2][0] = -dt * Omega_Vector[1]; //-y
  Update_Matrix[2][1] = dt * Omega_Vector[0]; //x
  Update_Matrix[2][2] = 0;
#else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -dt * ahrs_dcm.imu_rate.r; //-z
  Update_Matrix[0][2] = dt * ahrs_dcm.imu_rate.q; //y
  Update_Matrix[1][0] = dt * ahrs_dcm.imu_rate.r; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -dt * ahrs_dcm.imu_rate.p;
  Update_Matrix[2][0] = -dt * ahrs_dcm.imu_rate.q;
  Update_Matrix[2][1] = dt * ahrs_dcm.imu_rate.p;
  Update_Matrix[2][2] = 0;
#endif

  Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

  for (int x = 0; x < 3; x++) { //Matrix Addition (update)
    for (int y = 0; y < 3; y++) {
      DCM_Matrix[x][y] += Temporary_Matrix[x][y];
    }
  }
}

static void compute_ahrs_representations(void)
{
#if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
  ahrs_dcm.ltp_to_imu_euler.phi = atan2(accel_float.y, accel_float.z);   // atan2(acc_y,acc_z)
  ahrs_dcm.ltp_to_imu_euler.theta = -asin((accel_float.x) / GRAVITY); // asin(acc_x)
  ahrs_dcm.ltp_to_imu_euler.psi = 0;
#else
  ahrs_dcm.ltp_to_imu_euler.phi = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
  ahrs_dcm.ltp_to_imu_euler.theta = -asin(DCM_Matrix[2][0]);
  ahrs_dcm.ltp_to_imu_euler.psi = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
  ahrs_dcm.ltp_to_imu_euler.psi += M_PI; // Rotating the angle 180deg to fit for PPRZ
#endif
}

void ahrs_dcm_set_body_to_imu(struct OrientationReps *body_to_imu)
{
  ahrs_dcm_set_body_to_imu_quat(orientationGetQuat_f(body_to_imu));
}

void ahrs_dcm_set_body_to_imu_quat(struct FloatQuat *q_b2i)
{
  orientationSetQuat_f(&ahrs_dcm.body_to_imu, q_b2i);

  if (!ahrs_dcm.is_aligned) {
    /* Set ltp_to_imu so that body is zero */
    ahrs_dcm.ltp_to_imu_euler = *orientationGetEulers_f(&ahrs_dcm.body_to_imu);
  }
}
