
#include "math/pprz_algebra_float.h"

// Inputs for DCM
extern float Gyro_Vector[3];
extern float Accel_Vector[3];

// Integrate Inertial
void Matrix_update(void);
void Normalize(void);

// Input GPS/Pressure/Magnetometer data

// Correct
void Drift_correction(void);

// Get outputs
void Euler_angles(void);
extern struct FloatEulers euler;

// DCM Parameters

//#define Kp_ROLLPITCH 0.2
#define Kp_ROLLPITCH 0.015
#define Ki_ROLLPITCH 0.000010
#define Kp_YAW 1.2      	//High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005

#define GRAVITY 9.81


#define OUTPUTMODE 1
// Mode 0 = DCM integration without Ki gyro bias
// Mode 1 = DCM integration with Kp and Ki
// Mode 2 = direct accelerometer -> euler

#define MAGNETOMETER 1
extern float MAG_Heading;

#define PERFORMANCE_REPORTING 1
#if PERFORMANCE_REPORTING == 1
extern int renorm_sqrt_count;
extern int renorm_blowup_count;
extern float imu_health;
#endif



