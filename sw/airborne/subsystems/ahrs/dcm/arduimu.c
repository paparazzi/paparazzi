
#include <math.h>

#include "arduimu.h"
#include "vector.h"
#include "dcm.h"

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

float G_Dt=0.05;

float AN[8]; //array that store the 6 ADC filtered data
float AN_OFFSET[8]; //Array that stores the Offset of the gyros

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};



unsigned int cycleCount=0;
uint8_t gyro_sat=0;

float DCM_Matrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}};
float Update_Matrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

 // Performance Monitoring variables
unsigned int imu_health = 65012;


