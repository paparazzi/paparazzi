void Euler_angles(void);
void Matrix_update(void);
void Accel_adjust(void);
void Drift_correction(void);
void Normalize(void);

/** defines for euler vector */
enum euler_idx_t { EULER_ROLL, EULER_PITCH, EULER_YAW, EULER_LAST };

#define M_PI    3.14159265358979323846

// variables

/** output vector with angles in rad */
extern float euler[EULER_LAST];
