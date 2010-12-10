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

/** \file ahrs_float_dcm.c
 *  \brief Attitude estimation for fixedwings based on the DCM
 *
 */

#include "std.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_float_dcm.h"
#include "subsystems/ahrs/ahrs_float_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/imu.h"

#include "subsystems/ahrs/ahrs_float_dcm_algebra.h"

#include <string.h>

//FIXME this is still needed for fixedwing integration
#include "estimator.h"

struct AhrsFloatDCM ahrs_impl;

// remotely settable
float imu_roll_neutral = RadOfDeg(IMU_ROLL_NEUTRAL_DEFAULT);
float imu_pitch_neutral = RadOfDeg(IMU_PITCH_NEUTRAL_DEFAULT);

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

// DCM Working variables
float G_Dt=0.05;

struct FloatVect3 accel_float = {0,0,0};

float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};		//Omega Proportional correction
float Omega_I[3]= {0,0,0};		//Omega Integrator
float Omega[3]= {0,0,0};

float DCM_Matrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}};
float Update_Matrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

float speed_3d = 0;

static inline void compute_body_orientation_and_rates(void);
void Normalize(void);
void Drift_correction(void);
void Euler_angles(void);
void Matrix_update(void);

/**************************************************/

void ahrs_update_fw_estimator( void )
{
  Euler_angles();

  //warning, only eulers written to ahrs struct so far
  //compute_body_orientation_and_rates();

  // export results to estimator
  estimator_phi   = ahrs_float.ltp_to_imu_euler.phi - imu_roll_neutral;
  estimator_theta = ahrs_float.ltp_to_imu_euler.theta - imu_pitch_neutral;
  estimator_psi   = ahrs_float.ltp_to_imu_euler.psi;
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
}

void ahrs_align(void)
{
  /* Compute an initial orientation using euler angles */
  ahrs_float_get_euler_from_accel_mag(&ahrs_float.ltp_to_imu_euler, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);

  /* Convert initial orientation in quaternion and rotation matrice representations. */
  FLOAT_QUAT_OF_EULERS(ahrs_float.ltp_to_imu_quat, ahrs_float.ltp_to_imu_euler);
  FLOAT_RMAT_OF_QUAT(ahrs_float.ltp_to_imu_rmat, ahrs_float.ltp_to_imu_quat);

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

  Matrix_update();
  Normalize();
  //INFO, ahrs struct only updated in ahrs_update_fw_estimator
}

void ahrs_update_accel(void)
{
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);

  //FIXME
  /*if (gps_mode==3) {    //Remove centrifugal acceleration.
    accel_float.y += speed_3d*Omega[2];  // Centrifugal force on Acc_y = GPS_speed*GyroZ
    accel_float.z -= speed_3d*Omega[1];  // Centrifugal force on Acc_z = GPS_speed*GyroY
  }
  */

  Drift_correction();
}

void ahrs_update_mag(void)
{
  //TODO
}

void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  boolean problem=FALSE;

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
      DCM_Matrix[0][0]= 1.0f;
      DCM_Matrix[0][1]= 0.0f;
      DCM_Matrix[0][2]= 0.0f;
      DCM_Matrix[1][0]= 0.0f;
      DCM_Matrix[1][1]= 1.0f;
      DCM_Matrix[1][2]= 0.0f;
      DCM_Matrix[2][0]= 0.0f;
      DCM_Matrix[2][1]= 0.0f;
      DCM_Matrix[2][2]= 1.0f;
      problem = FALSE;
  }
}

/**************************************************/
//FIXME
/*  extern short gps_course;
  extern short gps_gspeed;
  extern short gps_climb;
  extern short gps_mode;
*/
#ifdef USE_MAGNETOMETER
float MAG_Heading;
#endif


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
  float ground_speed; // This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
  float ground_course; //This is the runaway direction of you "plane" in degrees
  float COGX; //Course overground X axis
  float COGY; //Course overground Y axis

  // hwarm
  /* FIXME
    ground_course=gps_course/10.-180.;
    ground_speed=gps_gspeed/100.;
    float ground_climb=gps_climb/100.;
    speed_3d = sqrt(ground_speed*ground_speed+ground_climb*ground_climb);
  */
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

  #if USE_MAGNETOMETER==1
    // We make the gyro YAW drift correction based on compass magnetic heading
    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  #else  // Use GPS Ground course to correct yaw gyro drift
    /* FIXME
    if(gps_mode==3 && ground_speed>= 0.5) {  //hwarm
      COGX = cos(RadOfDeg(ground_course));
      COGY = sin(RadOfDeg(ground_course));
      errorCourse=(DCM_Matrix[0][0]*COGY) - (DCM_Matrix[1][0]*COGX);  //Calculating YAW error
      Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

      Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
      Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

      Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
      Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
    }
    */
  #endif
  //  Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
  Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I,Omega_I));
  if (Integrator_magnitude > DegOfRad(300)) {
    Vector_Scale(Omega_I,Omega_I,0.5f*DegOfRad(300)/Integrator_magnitude);
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

void Euler_angles(void)
{
#if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
  ahrs_float.ltp_to_imu_euler.phi = atan2(Accel_Vector[1],Accel_Vector[2]);    // atan2(acc_y,acc_z)
  ahrs_float.ltp_to_imu_euler.theta = -asin((Accel_Vector[0])/GRAVITY); // asin(acc_x)
  ahrs_float.ltp_to_imu_euler.psi = 0;
#else
  ahrs_float.ltp_to_imu_euler.phi = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  ahrs_float.ltp_to_imu_euler.theta = -asin(DCM_Matrix[2][0]);
  ahrs_float.ltp_to_imu_euler.psi = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
  ahrs_float.ltp_to_imu_euler.psi += M_PI; // Rotating the angle 180deg to fit for PPRZ
#endif
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
