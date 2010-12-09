#include <math.h>

#include "wiring.h"
#include "vector.h"
#include "matrix.h"
#include "arduimu.h"

#ifdef ANALOG_IMU
#include "analogimu.h"
#include "analogimu_util.h"
#endif // ANALOG_IMU

#include "dcm.h"
/**
*	Geschätzte Winkel in der Eulerdarstellung
*	Estimated angles as Euler
*/
float euler[EULER_LAST] = {0.};

/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  boolean problem=FALSE;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= Vector_Dot_Product(&temporary[0][0],&temporary[0][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);
#if PERFORMANCE_REPORTING == 1  
    renorm_sqrt_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???SQT:1,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  } else {
    problem = TRUE;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
	#if PRINT_DEBUG != 0
    Serial.print("???PRB:1,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
      Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[1][0],&temporary[1][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);  
#if PERFORMANCE_REPORTING == 1    
    renorm_sqrt_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???SQT:2,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  } else {
    problem = TRUE;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???PRB:2,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[2][0],&temporary[2][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);   
#if PERFORMANCE_REPORTING == 1 
    renorm_sqrt_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???SQT:3,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  } else {
    problem = TRUE;  
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???PRB:3,RNM:");
    Serial.print (renorm);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
  
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
  extern short gps_course;
  extern short gps_gspeed;
  extern short gps_climb;
  extern short gps_mode;

void Drift_correction(void)
{
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  float Integrator_magnitude;
  // hwarm
    ground_course=gps_course/10.-180.;
    ground_speed=gps_gspeed/100.;
    float ground_climb=gps_climb/100.;
    speed_3d= sqrt(ground_speed*ground_speed+ground_climb*ground_climb);
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
  
  #if PERFORMANCE_REPORTING == 1
    tempfloat = ((Accel_weight - 0.5) * 256.0f);    //amount added was determined to give imu_health a time constant about twice the time constant of the roll/pitch drift correction
    imu_health += tempfloat;
    imu_health = constrain(imu_health,129,65405);
  #endif
  
  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
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
    if(gps_mode==3 && ground_speed>= 0.5)  //hwarm
  {

    COGX = cos(ToRad(ground_course));
    COGY = sin(ToRad(ground_course));
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
  if (Integrator_magnitude > ToRad(300)) {
    Vector_Scale(Omega_I,Omega_I,0.5f*ToRad(300)/Integrator_magnitude);
#if PRINT_DEBUG != 0
    Serial.print("!!!INT:1,MAG:");
    Serial.print (ToDeg(Integrator_magnitude));

    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
  
  
}
/**************************************************/
void Accel_adjust(void)
{
  #ifndef ANALOGIMU_ROTATED
    Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
    Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
  #else
    Accel_Vector[0] -= Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_x = GPS_speed*GyroZ (ok, wenn x beim rollen nach rechts negativ)
    Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[0]);  // Centrifugal force on Acc_z = GPS_speed*GyroX (ok, wenn nase hoch positiv)
  #endif
}
/**************************************************/

void Matrix_update(void)
{
  /* Offset is set dynamic on Ground*/
  Gyro_Vector[0]= -gyro_to_zero[G_ROLL]   + gyro[G_ROLL];
  Gyro_Vector[1]= -gyro_to_zero[G_PITCH]  + gyro[G_PITCH];
  Gyro_Vector[2]= -gyro_to_zero[G_PITCH]  + gyro[G_YAW];
  
  Accel_Vector[0] = accel[ACC_X];
  Accel_Vector[1] = accel[ACC_Y];
  Accel_Vector[2] = accel[ACC_Z];
  
  
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
//#define USE_GPS
#ifdef USE_GPS
 if (gps_mode==3) Accel_adjust();    //Remove centrifugal acceleration.
#endif
  
#define OUTPUTMODE 1
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
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
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
  //#define OUTPUTMODE 2
  #if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
    euler[EULER_ROLL] = atan2(Accel_Vector[1],Accel_Vector[2]);    // atan2(acc_y,acc_z)
    //euler[EULER_PITCH] = -asin((Accel_Vector[0])/(double)GRAVITY); // asin(acc_x)
    //todo: chni:ordentlich lösen!
    euler[EULER_PITCH] = -asin((Accel_Vector[0])/9.81); // asin(acc_x)
    euler[EULER_YAW] = 0;
  #else
    //pitch = -asin(DCM_Matrix[2][0]);
    //roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    //yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
    #ifndef ANALOGIMU_ROTATED
     euler[EULER_PITCH] = -asin(DCM_Matrix[2][0]);
     euler[EULER_ROLL] = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    #else
     euler[EULER_ROLL] = -asin(DCM_Matrix[2][0]);
     euler[EULER_PITCH] = -atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    #endif
     
    euler[EULER_YAW] = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
    euler[EULER_YAW] += M_PI; // Rotating the angle 180deg to fit for PPRZ
  #endif
}

