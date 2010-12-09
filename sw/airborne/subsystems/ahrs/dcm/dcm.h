
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
enum euler_idx_t { EULER_ROLL, EULER_PITCH, EULER_YAW, EULER_LAST };
extern float euler[3];

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



