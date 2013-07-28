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

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_float_dcm.h"
#include "subsystems/ahrs/ahrs_float_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/imu.h"
#include "firmwares/fixedwing/autopilot.h"	// launch detection

#include "subsystems/ahrs/ahrs_float_dcm_algebra.h"
#include "math/pprz_algebra_float.h"

#include "state.h"

#if USE_GPS
#include "subsystems/gps.h"
#endif

#include <string.h>

#include "led.h"

#if FLOAT_DCM_SEND_DEBUG
// FIXME Debugging Only
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#endif

#ifndef AHRS_PROPAGATE_FREQUENCY
#define AHRS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif

// FIXME this is still needed for fixedwing integration
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;


struct AhrsFloatDCM ahrs_impl;

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

// DCM Working variables
const float G_Dt = 1. / ((float) AHRS_PROPAGATE_FREQUENCY );

struct FloatVect3 accel_float = {0,0,0};

float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};		//Omega Proportional correction
float Omega_I[3]= {0,0,0};		//Omega Integrator
float Omega[3]= {0,0,0};

float DCM_Matrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}};
float Update_Matrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

#if USE_MAGNETOMETER
float MAG_Heading_X = 1;
float MAG_Heading_Y = 0;
#endif

static inline void compute_ahrs_representations(void);
static inline void set_body_orientation_and_rates(void);
static inline void set_dcm_matrix_from_rmat(struct FloatRMat *rmat);

void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);

#if PERFORMANCE_REPORTING == 1
int renorm_sqrt_count = 0;
int renorm_blowup_count = 0;
float imu_health = 0.;
#endif


static inline void set_dcm_matrix_from_rmat(struct FloatRMat *rmat)
{
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
      DCM_Matrix[i][j] = RMAT_ELMT(*rmat, j, i);
    }
  }
}


void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;

  /*
   * Initialises our IMU alignement variables
   * This should probably done in the IMU code instead
   */
  struct FloatEulers body_to_imu_euler =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  FLOAT_RMAT_OF_EULERS(ahrs_impl.body_to_imu_rmat, body_to_imu_euler);

  EULERS_COPY(ahrs_impl.ltp_to_imu_euler, body_to_imu_euler);

  FLOAT_RATES_ZERO(ahrs_impl.imu_rate);

  /* set inital filter dcm */
  set_dcm_matrix_from_rmat(&ahrs_impl.body_to_imu_rmat);

  ahrs_impl.gps_speed = 0;
  ahrs_impl.gps_acceleration = 0;
  ahrs_impl.gps_course = 0;
  ahrs_impl.gps_course_valid = FALSE;
  ahrs_impl.gps_age = 100;
}

void ahrs_align(void)
{
  /* Compute an initial orientation using euler angles */
  ahrs_float_get_euler_from_accel_mag(&ahrs_impl.ltp_to_imu_euler, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);

  /* Convert initial orientation in quaternion and rotation matrice representations. */
  struct FloatRMat ltp_to_imu_rmat;
  FLOAT_RMAT_OF_EULERS(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_euler);

  /* set filter dcm */
  set_dcm_matrix_from_rmat(&ltp_to_imu_rmat);

  /* Set initial body orientation */
  set_body_orientation_and_rates();

  /* use averaged gyro as initial value for bias */
  struct Int32Rates bias0;
  RATES_COPY(bias0, ahrs_aligner.lp_gyro);
  RATES_FLOAT_OF_BFP(ahrs_impl.gyro_bias, bias0);

  ahrs.status = AHRS_RUNNING;
}


void ahrs_propagate(void)
{
  /* convert imu data to floating point */
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro);

  /* unbias rate measurement */
  RATES_DIFF(ahrs_impl.imu_rate, gyro_float, ahrs_impl.gyro_bias);

  /* Uncouple Motions */
#ifdef IMU_GYRO_P_Q
  float dp=0,dq=0,dr=0;
  dp += ahrs_impl.imu_rate.q * IMU_GYRO_P_Q;
  dp += ahrs_impl.imu_rate.r * IMU_GYRO_P_R;
  dq += ahrs_impl.imu_rate.p * IMU_GYRO_Q_P;
  dq += ahrs_impl.imu_rate.r * IMU_GYRO_Q_R;
  dr += ahrs_impl.imu_rate.p * IMU_GYRO_R_P;
  dr += ahrs_impl.imu_rate.q * IMU_GYRO_R_Q;

  ahrs_impl.imu_rate.p += dp;
  ahrs_impl.imu_rate.q += dq;
  ahrs_impl.imu_rate.r += dr;
#endif

  Matrix_update();

  Normalize();

  compute_ahrs_representations();
}

void ahrs_update_gps(void)
{
  static float last_gps_speed_3d = 0;

#if USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    ahrs_impl.gps_age = 0;
    ahrs_impl.gps_speed = gps.speed_3d/100.;

    if(gps.gspeed >= 500) { //got a 3d fix and ground speed is more than 0.5 m/s
      ahrs_impl.gps_course = ((float)gps.course)/1.e7;
      ahrs_impl.gps_course_valid = TRUE;
    } else {
      ahrs_impl.gps_course_valid = FALSE;
    }
  } else {
    ahrs_impl.gps_age = 100;
  }
#endif

  ahrs_impl.gps_acceleration += (   ((ahrs_impl.gps_speed - last_gps_speed_3d)*4.0f)  - ahrs_impl.gps_acceleration) / 5.0f;
  last_gps_speed_3d = ahrs_impl.gps_speed;
}


void ahrs_update_accel(void)
{
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);

  // DCM filter uses g-force as positive
  // accelerometer measures [0 0 -g] in a static case
  accel_float.x = -accel_float.x;
  accel_float.y = -accel_float.y;
  accel_float.z = -accel_float.z;


  ahrs_impl.gps_age ++;
  if (ahrs_impl.gps_age < 50) {    //Remove centrifugal acceleration and longitudinal acceleration
#if USE_AHRS_GPS_ACCELERATIONS
PRINT_CONFIG_MSG("AHRS_FLOAT_DCM uses GPS acceleration.")
    accel_float.x += ahrs_impl.gps_acceleration;      // Longitudinal acceleration
#endif
    accel_float.y += ahrs_impl.gps_speed * Omega[2];  // Centrifugal force on Acc_y = GPS_speed*GyroZ
    accel_float.z -= ahrs_impl.gps_speed * Omega[1];  // Centrifugal force on Acc_z = GPS_speed*GyroY
  }
  else
  {
    ahrs_impl.gps_speed = 0;
    ahrs_impl.gps_acceleration = 0;
    ahrs_impl.gps_age = 100;
  }

  Drift_correction();
}


void ahrs_update_mag(void)
{
#if USE_MAGNETOMETER
#warning MAGNETOMETER FEEDBACK NOT TESTED YET

  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cosf(ahrs_impl.ltp_to_imu_euler.phi);
  sin_roll = sinf(ahrs_impl.ltp_to_imu_euler.phi);
  cos_pitch = cosf(ahrs_impl.ltp_to_imu_euler.theta);
  sin_pitch = sinf(ahrs_impl.ltp_to_imu_euler.theta);


  // Pitch&Roll Compensation:
  MAG_Heading_X = imu.mag.x*cos_pitch+imu.mag.y*sin_roll*sin_pitch+imu.mag.z*cos_roll*sin_pitch;
  MAG_Heading_Y = imu.mag.y*cos_roll-imu.mag.z*sin_roll;

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

  struct FloatVect3 ltp_mag;

  ltp_mag.x = MAG_Heading_X;
  ltp_mag.y = MAG_Heading_Y;

#if FLOAT_DCM_SEND_DEBUG
  // Downlink
  RunOnceEvery(10,DOWNLINK_SEND_IMU_MAG(DefaultChannel, DefaultDevice, &ltp_mag.x, &ltp_mag.y, &ltp_mag.z));
#endif

  // Magnetic Heading
  // MAG_Heading = atan2(imu.mag.y, -imu.mag.x);
#endif
}

void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  uint8_t problem=FALSE;

  // Find the non-orthogonality of X wrt Y
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  // Add half the XY error to X, and half to Y
  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error);           //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error);           //eq.19
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);  //eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);  //eq.19

  // The third axis is simply set perpendicular to the first 2. (there is not correction of XY based on Z)
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

  // Normalize lenght of X
  renorm= Vector_Dot_Product(&temporary[0][0],&temporary[0][0]);
  // a) if norm is close to 1, use the fast 1st element from the tailer expansion of SQRT
  // b) if the norm is further from 1, use a real sqrt
  // c) norm is huge: disaster! reset! mayday!
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                          //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);
#if PERFORMANCE_REPORTING == 1
    renorm_sqrt_count++;
#endif
  } else {
    problem = TRUE;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
  }
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

  // Normalize lenght of Y
  renorm= Vector_Dot_Product(&temporary[1][0],&temporary[1][0]);
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);
#if PERFORMANCE_REPORTING == 1
    renorm_sqrt_count++;
#endif
  } else {
    problem = TRUE;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
  }
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

  // Normalize lenght of Z
  renorm= Vector_Dot_Product(&temporary[2][0],&temporary[2][0]);
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);
#if PERFORMANCE_REPORTING == 1
    renorm_sqrt_count++;
#endif
  } else {
    problem = TRUE;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
  }
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);

  // Reset on trouble
  if (problem) {                // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
    set_dcm_matrix_from_rmat(&ahrs_impl.body_to_imu_rmat);
    problem = FALSE;
  }
}


void Drift_correction(void)
{
  //Compensation the Roll, Pitch and Yaw drift.
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  float Integrator_magnitude;

  // Local Working Variables
  float errorRollPitch[3];
  float errorYaw[3];
  float errorCourse;

  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(accel_float.x*accel_float.x + accel_float.y*accel_float.y + accel_float.z*accel_float.z);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = Chop(1 - 2*fabs(1 - Accel_magnitude),0,1);  //


  #if PERFORMANCE_REPORTING == 1
  {

    float tempfloat = ((Accel_weight - 0.5) * 256.0f);    //amount added was determined to give imu_health a time constant about twice the time constant of the roll/pitch drift correction
    imu_health += tempfloat;
    Bound(imu_health,129,65405);
  }
  #endif

  Vector_Cross_Product(&errorRollPitch[0],&accel_float.x,&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);

  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);

  //*****YAW***************

#if USE_MAGNETOMETER
  // We make the gyro YAW drift correction based on compass magnetic heading
//  float mag_heading_x = cos(MAG_Heading);
//  float mag_heading_y = sin(MAG_Heading);
  // 2D dot product
  errorCourse=(DCM_Matrix[0][0]*MAG_Heading_Y) + (DCM_Matrix[1][0]*MAG_Heading_X);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I

#else // Use GPS Ground course to correct yaw gyro drift

  if (ahrs_impl.gps_course_valid) {
    float course = ahrs_impl.gps_course - M_PI; //This is the runaway direction of you "plane" in rad
    float COGX = cosf(course); //Course overground X axis
    float COGY = sinf(course); //Course overground Y axis

    errorCourse=(DCM_Matrix[0][0]*COGY) - (DCM_Matrix[1][0]*COGX);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  }
#if USE_MAGNETOMETER_ONGROUND == 1
PRINT_CONFIG_MSG("AHRS_FLOAT_DCM uses magnetometer prior to takeoff and GPS during flight")
  else if (launch == FALSE)
  {
    float COGX = imu.mag.x; // Non-Tilt-Compensated (for filter stability reasons)
    float COGY = imu.mag.y; // Non-Tilt-Compensated (for filter stability reasons)

    errorCourse=(DCM_Matrix[0][0]*COGY) - (DCM_Matrix[1][0]*COGX);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    // P only
    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW / 10.0);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.fi
  }
#endif // USE_MAGNETOMETER_ONGROUND
#endif

  //  Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
  Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I,Omega_I));
  if (Integrator_magnitude > RadOfDeg(300)) {
    Vector_Scale(Omega_I,Omega_I,0.5f*RadOfDeg(300)/Integrator_magnitude);
  }


}
/**************************************************/

void Matrix_update(void)
{
  Vector_Add(&Omega[0], &ahrs_impl.imu_rate.p, &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

 #if OUTPUTMODE==1    // With corrected data (drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*ahrs_impl.imu_rate.r;//-z
  Update_Matrix[0][2]=G_Dt*ahrs_impl.imu_rate.q;//y
  Update_Matrix[1][0]=G_Dt*ahrs_impl.imu_rate.r;//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*ahrs_impl.imu_rate.p;
  Update_Matrix[2][0]=-G_Dt*ahrs_impl.imu_rate.q;
  Update_Matrix[2][1]=G_Dt*ahrs_impl.imu_rate.p;
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    }
  }
}

/*
 * Compute body orientation and rates from imu orientation and rates
 */
static inline void set_body_orientation_and_rates(void) {

  struct FloatRates body_rate;
  FLOAT_RMAT_TRANSP_RATEMULT(body_rate, ahrs_impl.body_to_imu_rmat, ahrs_impl.imu_rate);
  stateSetBodyRates_f(&body_rate);

  struct FloatRMat ltp_to_imu_rmat, ltp_to_body_rmat;
  FLOAT_RMAT_OF_EULERS(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_euler);
  FLOAT_RMAT_COMP_INV(ltp_to_body_rmat, ltp_to_imu_rmat, ahrs_impl.body_to_imu_rmat);

  // Some stupid lines of code for neutrals
  struct FloatEulers ltp_to_body_euler;
  FLOAT_EULERS_OF_RMAT(ltp_to_body_euler, ltp_to_body_rmat);
  ltp_to_body_euler.phi -= ins_roll_neutral;
  ltp_to_body_euler.theta -= ins_pitch_neutral;
  stateSetNedToBodyEulers_f(&ltp_to_body_euler);

  // should be replaced at the end by:
  //   stateSetNedToBodyRMat_f(&ltp_to_body_rmat);

}

static inline void compute_ahrs_representations(void) {
#if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
  ahrs_impl.ltp_to_imu_euler.phi = atan2(accel_float.y,accel_float.z);    // atan2(acc_y,acc_z)
  ahrs_impl.ltp_to_imu_euler.theta = -asin((accel_float.x)/GRAVITY); // asin(acc_x)
  ahrs_impl.ltp_to_imu_euler.psi = 0;
#else
  ahrs_impl.ltp_to_imu_euler.phi = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  ahrs_impl.ltp_to_imu_euler.theta = -asin(DCM_Matrix[2][0]);
  ahrs_impl.ltp_to_imu_euler.psi = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
  ahrs_impl.ltp_to_imu_euler.psi += M_PI; // Rotating the angle 180deg to fit for PPRZ
#endif

  set_body_orientation_and_rates();

  /*
    RunOnceEvery(6,DOWNLINK_SEND_RMAT_DEBUG(DefaultChannel, DefaultDevice,
    &(DCM_Matrix[0][0]),
    &(DCM_Matrix[0][1]),
    &(DCM_Matrix[0][2]),

    &(DCM_Matrix[1][0]),
    &(DCM_Matrix[1][1]),
    &(DCM_Matrix[1][2]),

    &(DCM_Matrix[2][0]),
    &(DCM_Matrix[2][1]),
    &(DCM_Matrix[2][2])

    ));
  */
}

