#include <math.h>

#include "kalman.h"


struct KalmanPublic kalman_public;

static const float dt = ( 1024.0 * 256.0 ) / 16000000.0;

static float P_phi[2][2] = {
  { 1, 0 },
  { 0, 1 },
};

static const float      R_angle_phi = 1.3 * 1.3;
static const float      Q_angle_phi   = 0.001;
static const float      Q_gyro_phi  = 0.0075;

static float P_theta[2][2] = {
  { 1, 0 },
  { 0, 1 },
};

static const float      R_angle_theta = 1.3 * 1.3;
static const float      Q_angle_theta   = 0.001;
static const float      Q_gyro_theta  = 0.0075;

void kalman_state_update( float gyro_phi_measure, float gyro_theta_measure ) {
  const float unbiased_gyro_phi = gyro_phi_measure - kalman_public.gyro_phi_bias;
  const float P_phi_dot[2 * 2] = {
    Q_angle_phi - P_phi[0][1] - P_phi[1][0],/* 0,0 */
    - P_phi[1][1],                          /* 0,1 */
    - P_phi[1][1],                          /* 1,0 */
    Q_gyro_phi                              /* 1,1 */
  };
  kalman_public.phi_dot = unbiased_gyro_phi;
  kalman_public.phi += unbiased_gyro_phi * dt;
  P_phi[0][0] += P_phi_dot[0] * dt;
  P_phi[0][1] += P_phi_dot[1] * dt;
  P_phi[1][0] += P_phi_dot[2] * dt;
  P_phi[1][1] += P_phi_dot[3] * dt;

  const float unbiased_gyro_theta = gyro_theta_measure - kalman_public.gyro_theta_bias;
  const float P_theta_dot[2 * 2] = {
    Q_angle_theta - P_theta[0][1] - P_theta[1][0],/* 0,0 */
    - P_theta[1][1],                          /* 0,1 */
    - P_theta[1][1],                          /* 1,0 */
    Q_gyro_theta                              /* 1,1 */
  };
  kalman_public.theta_dot = unbiased_gyro_theta;
  kalman_public.theta += unbiased_gyro_theta * dt;
  P_theta[0][0] += P_theta_dot[0] * dt;
  P_theta[0][1] += P_theta_dot[1] * dt;
  P_theta[1][0] += P_theta_dot[2] * dt;
  P_theta[1][1] += P_theta_dot[3] * dt;

}


void kalman_kalman_update( float ax_measure, float ay_measure, float az_measure ) {
  const float angle_phi_measure = atan2( ay_measure, az_measure );
  const float angle_phi_err = angle_phi_measure - kalman_public.phi;
  const float             C_0 = 1;
  const float             PCt_0 = C_0 * P_phi[0][0]; /* + C_1 * P[0][1] = 0 */
  const float             PCt_1 = C_0 * P_phi[1][0]; /* + C_1 * P[1][1] = 0 */
  const float             E =
    R_angle_phi
    + C_0 * PCt_0
    /*      + C_1 * PCt_1 = 0 */
    ;
  const float             K_0 = PCt_0 / E;
  const float             K_1 = PCt_1 / E;
  const float             t_0 = PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] */
  const float             t_1 = C_0 * P_phi[0][1]; /* + C_1 * P[1][1]  = 0 */

  P_phi[0][0] -= K_0 * t_0;
  P_phi[0][1] -= K_0 * t_1;
  P_phi[1][0] -= K_1 * t_0;
  P_phi[1][1] -= K_1 * t_1;

  kalman_public.phi   += K_0 * angle_phi_err;
  kalman_public.gyro_phi_bias  += K_1 * angle_phi_err;


  const float angle_theta_measure = atan2( -ax_measure, az_measure );
  const float angle_theta_err = angle_theta_measure - kalman_public.theta;

  const float  t_C_0 = 1;
  const float t_PCt_0 = t_C_0 * P_theta[0][0];
  const float t_PCt_1 = t_C_0 * P_theta[1][0];
  const float t_E =
    R_angle_theta
    + t_C_0 * t_PCt_0
    /*      + C_1 * PCt_1 = 0 */
    ;
   ;
  const float             t_K_0 = t_PCt_0 / t_E;
  const float             t_K_1 = t_PCt_1 / t_E;
  const float             t_t_0 = t_PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] */
  const float             t_t_1 = t_C_0 * P_theta[0][1]; /* + C_1 * P[1][1]  = 0 */
  
  P_theta[0][0] -= t_K_0 * t_t_0;
  P_theta[0][1] -= t_K_0 * t_t_1;
  P_theta[1][0] -= t_K_1 * t_t_0;
  P_theta[1][1] -= t_K_1 * t_t_1;
 
  kalman_public.theta   += t_K_0 * angle_theta_err;
  kalman_public.gyro_theta_bias  += t_K_1 * angle_theta_err;


}
