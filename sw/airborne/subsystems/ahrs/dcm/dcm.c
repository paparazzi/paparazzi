
#include "std.h"

#include "dcm.h"

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise


// DCM Working variables
float G_Dt=0.05;

float Gyro_Vector[3] = {0,0,0};
float Accel_Vector[3] = {0,0,0};

float Omega_Vector[3]= {0,0,0}; 	//Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};		//Omega Proportional correction
float Omega_I[3]= {0,0,0};		//Omega Integrator
float Omega[3]= {0,0,0};

float DCM_Matrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}};
float Update_Matrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

struct FloatEulers euler= {0, 0, 0};
float speed_3d = 0;


/**************************************************/

// Algebra

static inline float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  return vector1[0]*vector2[0] + vector1[1]*vector2[1] + vector1[2]*vector2[2];
}

static inline void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

static inline void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  vectorOut[0]=vectorIn[0]*scale2;
  vectorOut[1]=vectorIn[1]*scale2;
  vectorOut[2]=vectorIn[2]*scale2;
}

static inline void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  vectorOut[0]=vectorIn1[0]+vectorIn2[0];
  vectorOut[1]=vectorIn1[1]+vectorIn2[1];
  vectorOut[2]=vectorIn1[2]+vectorIn2[2];
}

/*
#define Matrix_Multiply( _m_a2b, _m_b2c, _m_a2c) {			\
    _m_a2c[0] = (_m_b2c[0]*_m_a2b[0] + _m_b2c[1]*_m_a2b[3] + _m_b2c[2]*_m_a2b[6]); \
    _m_a2c[1] = (_m_b2c[0]*_m_a2b[1] + _m_b2c[1]*_m_a2b[4] + _m_b2c[2]*_m_a2b[7]); \
    _m_a2c[2] = (_m_b2c[0]*_m_a2b[2] + _m_b2c[1]*_m_a2b[5] + _m_b2c[2]*_m_a2b[8]); \
    _m_a2c[3] = (_m_b2c[3]*_m_a2b[0] + _m_b2c[4]*_m_a2b[3] + _m_b2c[5]*_m_a2b[6]); \
    _m_a2c[4] = (_m_b2c[3]*_m_a2b[1] + _m_b2c[4]*_m_a2b[4] + _m_b2c[5]*_m_a2b[7]); \
    _m_a2c[5] = (_m_b2c[3]*_m_a2b[2] + _m_b2c[4]*_m_a2b[5] + _m_b2c[5]*_m_a2b[8]); \
    _m_a2c[6] = (_m_b2c[6]*_m_a2b[0] + _m_b2c[7]*_m_a2b[3] + _m_b2c[8]*_m_a2b[6]); \
    _m_a2c[7] = (_m_b2c[6]*_m_a2b[1] + _m_b2c[7]*_m_a2b[4] + _m_b2c[8]*_m_a2b[7]); \
    _m_a2c[8] = (_m_b2c[6]*_m_a2b[2] + _m_b2c[7]*_m_a2b[5] + _m_b2c[8]*_m_a2b[8]); \
  }
*/

static inline void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
  float op[3];
  for(int x=0; x<3; x++)
  {
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
      {
        op[w]=a[x][w]*b[w][y];
      }
      mat[x][y]=op[0]+op[1]+op[2];
    }
  }
}





void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  boolean problem=FALSE;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error);           //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error);           //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);  //eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);  //eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= Vector_Dot_Product(&temporary[0][0],&temporary[0][0]); 
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
    ground_course=gps_course/10.-180.;
    ground_speed=gps_gspeed/100.;
    float ground_climb=gps_climb/100.;
    speed_3d = sqrt(ground_speed*ground_speed+ground_climb*ground_climb);
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
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

    COGX = cos(RadOfDeg(ground_course));
    COGY = sin(RadOfDeg(ground_course));
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
  if (Integrator_magnitude > DegOfRad(300)) {
    Vector_Scale(Omega_I,Omega_I,0.5f*DegOfRad(300)/Integrator_magnitude);
  }
  
  
}
/**************************************************/

void Matrix_update(void)
{
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  if (gps_mode==3)    //Remove centrifugal acceleration.
  {
    Accel_Vector[1] += speed_3d*Omega[2];  // Centrifugal force on Acc_y = GPS_speed*GyroZ
    Accel_Vector[2] -= speed_3d*Omega[1];  // Centrifugal force on Acc_z = GPS_speed*GyroY 
  }

  
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
  #if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
    euler.phi = atan2(Accel_Vector[1],Accel_Vector[2]);    // atan2(acc_y,acc_z)
    euler.theta = -asin((Accel_Vector[0])/GRAVITY); // asin(acc_x)
    euler.psi = 0;
  #else
    euler.phi = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    euler.theta = -asin(DCM_Matrix[2][0]);
    euler.psi = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
    euler.psi += M_PI; // Rotating the angle 180deg to fit for PPRZ
  #endif
}

