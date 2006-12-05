#ifndef AHRS_H
#define AHRS_H


#include <inttypes.h>

#define index_t uint8_t

/* our state */
extern float q0, q1, q2, q3;
extern float bias_p, bias_q, bias_r;
/* we maintain eulers angles */
extern float ahrs_phi, ahrs_theta, ahrs_psi;
/* we maintain unbiased rates */
extern float ahrs_p, ahrs_q, ahrs_r;

extern float P[7][7]; /* covariance */
extern float A[4][7]; /* jacobian   */

extern void ahrs_init( const int16_t *mag, const float* accel, const float* gyro );
extern void ahrs_predict( const float* gyro );

extern void ahrs_roll_update( const float* accel);
extern void ahrs_pitch_update( const float* accel);
extern void ahrs_yaw_update( const int16_t* mag);

#if 0
extern float ahrs_roll_of_accel( const float* accel_cal );
extern float ahrs_pitch_of_accel( const float* accel_cal);
extern float ahrs_yaw_of_mag( const int16_t *mag);
#endif

#endif /* AHRS_H */
