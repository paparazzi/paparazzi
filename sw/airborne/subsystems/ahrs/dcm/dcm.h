void Euler_angles(void);
void Accel_adjust(void);
void Drift_correction(void);
void Normalize(void);


// Inputs for DCM
extern float Gyro_Vector[3];
extern float Accel_Vector[3];

// Integrate Inertial
void Matrix_update(void);


// DCM Parameters

//#define Kp_ROLLPITCH 0.2
#define Kp_ROLLPITCH 0.015
#define Ki_ROLLPITCH 0.000010
#define Kp_YAW 1.2      	//High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005



/** defines for euler vector */
enum euler_idx_t { EULER_ROLL, EULER_PITCH, EULER_YAW, EULER_LAST };

#define M_PI    3.14159265358979323846
#define GRAVITY 9.81


/** output vector with angles in rad */
extern float euler[EULER_LAST];
