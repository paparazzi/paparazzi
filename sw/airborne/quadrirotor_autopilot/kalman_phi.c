#include <math.h>

#include "kalman.h"


struct KalmanPublic kalman_public;

static const float dt = ( 1024.0 * 256.0 ) / 16000000.0;

static float P[2][2] = {
  { 1, 0 },
  { 0, 1 },
};

static const float      R_angle_phi = 0.3;

static const float      Q_angle_phi   = 0.001;
static const float      Q_gyro_phi  = 0.003;


void kalman_state_update( float gyro_phi_measure ) {
  const float unbiased_gyro_phi = gyro_phi_measure - kalman_public.gyro_phi_bias;
  const float Pdot[2 * 2] = {
    Q_angle_phi - P[0][1] - P[1][0],/* 0,0 */
    - P[1][1],                      /* 0,1 */
    - P[1][1],                      /* 1,0 */
    Q_gyro_phi                      /* 1,1 */
  };
  kalman_public.phi_dot = unbiased_gyro_phi;
  kalman_public.phi += unbiased_gyro_phi * dt;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;

}


void kalman_kalman_update( float ax_measure, float ay_measure ) {
  const float angle_phi_measure = atan2( ax_measure, ay_measure );
  const float angle_phi_err = angle_phi_measure - kalman_public.phi;
  const float             C_0 = 1;
  const float             PCt_0 = C_0 * P[0][0]; /* + C_1 * P[0][1] = 0 */
  const float             PCt_1 = C_0 * P[1][0]; /* + C_1 * P[1][1] = 0 */
  const float             E =
    R_angle_phi
    + C_0 * PCt_0
    /*      + C_1 * PCt_1 = 0 */
    ;
  const float             K_0 = PCt_0 / E;
  const float             K_1 = PCt_1 / E;
  const float             t_0 = PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] */
  const float             t_1 = C_0 * P[0][1]; /* + C_1 * P[1][1]  = 0 */

  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;

  kalman_public.phi   += K_0 * angle_phi_err;
  kalman_public.gyro_phi_bias  += K_1 * angle_phi_err;
}
