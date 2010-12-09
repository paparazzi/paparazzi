
#include <inttypes.h>

extern float G_Dt;

extern float Temporary_Matrix[3][3];
extern float Update_Matrix[3][3];
extern float DCM_Matrix[3][3];
extern unsigned int cycleCount;
extern float Omega[3];
extern float Omega_I[3];
extern float Omega_P[3];
extern float Omega_Vector[3];
extern float Gyro_Vector[3];
extern float Accel_Vector[3];
extern float AN_OFFSET[8];
extern float AN[8];

#define GRAVITY 9.81

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi


//#define Kp_ROLLPITCH 0.2
#define Kp_ROLLPITCH 0.015
#define Kp_YAW 1.2      //High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005
#define Ki_ROLLPITCH 0.000010

#define SPEEDFILT 2 // >1 use min speed filter for yaw drift cancellation, 0=do not use speed filter

/* Support for optional magnetometer (1 enabled, 0 dissabled) */
#define USE_MAGNETOMETER 0 // use 1 if you want to make yaw gyro drift corrections using the optional magnetometer


void dcm_init( int i );


