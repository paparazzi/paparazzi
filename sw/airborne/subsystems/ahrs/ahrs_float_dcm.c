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

/** \file ahrs_float_dcm.c
 *  \brief Attitude estimation for fixedwings based on the DCM
 *  Theory: http://code.google.com/p/gentlenav/downloads/list  file DCMDraft2.pdf
 *
 */

#include "std.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_float_dcm.h"
#include "subsystems/ahrs/ahrs_float_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/imu.h"

#include "subsystems/ahrs/ahrs_float_dcm_algebra.h"
#include "math/pprz_algebra_float.h"

#ifdef USE_GPS
#include "subsystems/gps.h"
#endif

#include <string.h>

// FIXME this is still needed for fixedwing integration
#include "estimator.h"
#include "led.h"

// FIXME Debugging Only
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"


struct AhrsFloatDCM ahrs_impl;

// remotely settable
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;

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

#ifdef USE_MAGNETOMETER
float MAG_Heading_X = 1;
float MAG_Heading_Y = 0;
#endif

static inline void compute_body_orientation_and_rates(void);
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


/**************************************************/

void ahrs_update_fw_estimator( void )
{
#if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
  ahrs_float.ltp_to_imu_euler.phi = atan2(accel_float.y,accel_float.z);    // atan2(acc_y,acc_z)
  ahrs_float.ltp_to_imu_euler.theta = -asin((accel_float.x)/GRAVITY); // asin(acc_x)
  ahrs_float.ltp_to_imu_euler.psi = 0;
#else
  ahrs_float.ltp_to_imu_euler.phi = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  ahrs_float.ltp_to_imu_euler.theta = -asin(DCM_Matrix[2][0]);
  ahrs_float.ltp_to_imu_euler.psi = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
  ahrs_float.ltp_to_imu_euler.psi += M_PI; // Rotating the angle 180deg to fit for PPRZ
#endif

  /* set quaternion and rotation matrix representations as well */
  FLOAT_QUAT_OF_EULERS(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_euler);
  FLOAT_RMAT_OF_EULERS(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_euler);

  compute_body_orientation_and_rates();

  // export results to estimator
  estimator_phi   = ahrs_float.ltp_to_body_euler.phi - ins_roll_neutral;
  estimator_theta = ahrs_float.ltp_to_body_euler.theta - ins_pitch_neutral;
  estimator_psi   = ahrs_float.ltp_to_body_euler.psi;

  estimator_p = ahrs_float.body_rate.p;
  estimator_q = ahrs_float.body_rate.q;
/*
  RunOnceEvery(6,DOWNLINK_SEND_RMAT_DEBUG(DefaultChannel,
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


void ahrs_init(void) {
  ahrs_float.status = AHRS_UNINIT;

  /*
   * Initialises our IMU alignement variables
   * This should probably done in the IMU code instead
   */
  struct FloatEulers body_to_imu_euler =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  FLOAT_QUAT_OF_EULERS(ahrs_impl.body_to_imu_quat, body_to_imu_euler);
  FLOAT_RMAT_OF_EULERS(ahrs_impl.body_to_imu_rmat, body_to_imu_euler);

  /* set ltp_to_body to zero */
  FLOAT_QUAT_ZERO(ahrs_float.ltp_to_body_quat);
  FLOAT_EULERS_ZERO(ahrs_float.ltp_to_body_euler);
  FLOAT_RMAT_ZERO(ahrs_float.ltp_to_body_rmat);
  FLOAT_RATES_ZERO(ahrs_float.body_rate);

  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_float.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  RMAT_COPY(ahrs_float.ltp_to_imu_rmat, ahrs_impl.body_to_imu_rmat);
  EULERS_COPY(ahrs_float.ltp_to_imu_euler, body_to_imu_euler);
  FLOAT_RATES_ZERO(ahrs_float.imu_rate);

  /* set inital filter dcm */
  set_dcm_matrix_from_rmat(&ahrs_float.ltp_to_imu_rmat);
}

void ahrs_align(void)
{
  /* Compute an initial orientation using euler angles */
  ahrs_float_get_euler_from_accel_mag(&ahrs_float.ltp_to_imu_euler, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);

  /* Convert initial orientation in quaternion and rotation matrice representations. */
  FLOAT_QUAT_OF_EULERS(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_euler);
  FLOAT_RMAT_OF_QUAT(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_quat);

  /* set filter dcm */
  set_dcm_matrix_from_rmat(&ahrs_float.ltp_to_imu_rmat);

  /* Compute initial body orientation */
  compute_body_orientation_and_rates();

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
  RATES_DIFF(ahrs_float.imu_rate, gyro_float, ahrs_impl.gyro_bias);

  /* Uncouple Motions */
#ifdef IMU_GYRO_P_Q
  float dp=0,dq=0,dr=0;
  dp += ahrs_float.imu_rate.q * IMU_GYRO_P_Q;
  dp += ahrs_float.imu_rate.r * IMU_GYRO_P_R;
  dq += ahrs_float.imu_rate.p * IMU_GYRO_Q_P;
  dq += ahrs_float.imu_rate.r * IMU_GYRO_Q_R;
  dr += ahrs_float.imu_rate.p * IMU_GYRO_R_P;
  dr += ahrs_float.imu_rate.q * IMU_GYRO_R_Q;

  ahrs_float.imu_rate.p += dp;
  ahrs_float.imu_rate.q += dq;
  ahrs_float.imu_rate.r += dr;
#endif

  Matrix_update();
  // INFO, ahrs struct only updated in ahrs_update_fw_estimator

  Normalize();
}

void ahrs_update_accel(void)
{

  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);

  // DCM filter uses g-force as positive
  // accelerometer measures [0 0 -g] in a static case
  accel_float.x = -accel_float.x;
  accel_float.y = -accel_float.y;
  accel_float.z = -accel_float.z;


#ifdef USE_GPS
  if (gps.fix == GPS_FIX_3D) {    //Remove centrifugal acceleration.
    accel_float.y += gps.speed_3d/100. * Omega[2];  // Centrifugal force on Acc_y = GPS_speed*GyroZ
    accel_float.z -= gps.speed_3d/100. * Omega[1];  // Centrifugal force on Acc_z = GPS_speed*GyroY
  }
#endif

  Drift_correction();
}


void ahrs_update_mag(void)
{
#ifdef USE_MAGNETOMETER
#warning MAGNETOMETER FEEDBACK NOT TESTED YET

  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(ahrs_float.ltp_to_imu_euler.phi);
  sin_roll = sin(ahrs_float.ltp_to_imu_euler.phi);
  cos_pitch = cos(ahrs_float.ltp_to_imu_euler.theta);
  sin_pitch = sin(ahrs_float.ltp_to_imu_euler.theta);


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

  // Downlink
  RunOnceEvery(10,DOWNLINK_SEND_IMU_MAG(DefaultChannel, &ltp_mag.x, &ltp_mag.y, &ltp_mag.z));

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

#ifdef USE_MAGNETOMETER
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

#elif defined USE_GPS // Use GPS Ground course to correct yaw gyro drift

  if(gps.fix == GPS_FIX_3D && gps.gspeed>= 500) { //got a 3d fix and ground speed is more than 0.5 m/s
    float ground_course = ((float)gps.course)/1.e7 - M_PI; //This is the runaway direction of you "plane" in rad
    float COGX = cosf(ground_course); //Course overground X axis
    float COGY = sinf(ground_course); //Course overground Y axis

    errorCourse=(DCM_Matrix[0][0]*COGY) - (DCM_Matrix[1][0]*COGX);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  }
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
  Vector_Add(&Omega[0], &ahrs_float.imu_rate.p, &Omega_I[0]);  //adding proportional term
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
  Update_Matrix[0][1]=-G_Dt*ahrs_float.imu_rate.r;//-z
  Update_Matrix[0][2]=G_Dt*ahrs_float.imu_rate.q;//y
  Update_Matrix[1][0]=G_Dt*ahrs_float.imu_rate.r;//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*ahrs_float.imu_rate.p;
  Update_Matrix[2][0]=-G_Dt*ahrs_float.imu_rate.q;
  Update_Matrix[2][1]=G_Dt*ahrs_float.imu_rate.p;
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
static inline void compute_body_orientation_and_rates(void) {

  FLOAT_QUAT_COMP_INV(ahrs_float.ltp_to_body_quat,
                      ahrs_float.ltp_to_imu_quat, ahrs_impl.body_to_imu_quat);
  FLOAT_RMAT_COMP_INV(ahrs_float.ltp_to_body_rmat,
                      ahrs_float.ltp_to_imu_rmat, ahrs_impl.body_to_imu_rmat);
  FLOAT_EULERS_OF_RMAT(ahrs_float.ltp_to_body_euler, ahrs_float.ltp_to_body_rmat);
  FLOAT_RMAT_TRANSP_RATEMULT(ahrs_float.body_rate, ahrs_impl.body_to_imu_rmat, ahrs_float.imu_rate);

}
