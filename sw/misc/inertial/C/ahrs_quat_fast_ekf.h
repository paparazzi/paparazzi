#ifndef AHRS_QUAT_FAST_EKF_H
#define AHRS_QUAT_FAST_EKF_H

#include <inttypes.h>
#include "6dof.h"

//#define FLOAT_T double
#define FLOAT_T float

/* ekf state : quaternion and gyro biases */
extern FLOAT_T afe_q0, afe_q1, afe_q2, afe_q3;
extern FLOAT_T afe_bias_p, afe_bias_q, afe_bias_r;
/* we maintain unbiased rates */
extern FLOAT_T afe_p, afe_q, afe_r;
/* we maintain eulers angles */
extern FLOAT_T afe_phi, afe_theta, afe_psi;

extern FLOAT_T afe_P[7][7]; /* covariance */

extern void afe_init( const int16_t *mag, const FLOAT_T* accel, const FLOAT_T* gyro );
extern void afe_predict( const FLOAT_T* gyro );
extern void afe_update_phi( const FLOAT_T* accel);
extern void afe_update_theta( const FLOAT_T* accel);
extern void afe_update_psi( const int16_t* mag);

#endif /* AHRS_QUAT_FAST_EKF_H_H */
