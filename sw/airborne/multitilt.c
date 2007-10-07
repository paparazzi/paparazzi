#include "multitilt.h"

#include <string.h>
#include <math.h>

#include "6dof.h"

uint8_t mtt_status;

#define DT 4e-3

/* attitude */
float mtt_phi;
float mtt_theta;
float mtt_psi;
/* unbiased rates */
float mtt_p;
float mtt_q;
float mtt_r;
/* gyro biases */
float mtt_bp;
float mtt_bq;
float mtt_br;
/* covariance matrix */
float mtt_P_phi[2][2];
float mtt_P_theta[2][2];
//float mtt_P_psi[2][2];

/* process covariance noise */
static const float Q_angle = 0.0005;
static const float Q_bias  = 0.0015;
/* Measurement covariance */
static const float R_accel = 0.3;

void multitilt_init(void) {
  mtt_status = MT_STATUS_UNINIT;
}

void multitilt_start(const float* accel, const float* gyro) {
  /* reset covariance matrices */
  const float cov_init[2][2] = {{1., 0.},
				{0., 1.}};
  memcpy(mtt_P_phi, cov_init, sizeof(cov_init));
  memcpy(mtt_P_theta, cov_init, sizeof(cov_init));
  //  memcpy(mtt_P_psi, cov_init, sizeof(cov_init));

  /* initialise state */
  mtt_p = 0.;
  mtt_q = 0.;
  mtt_r = 0.;
  
  mtt_bp = gyro[AXIS_P];
  mtt_bq = gyro[AXIS_Q];
  mtt_br = gyro[AXIS_R];

  const float init_phi = atan2(accel[AXIS_Y], accel[AXIS_Z]);
  const float g2 =					
      accel[AXIS_X]*accel[AXIS_X] +	
      accel[AXIS_Y]*accel[AXIS_Y] +	
      accel[AXIS_Z]*accel[AXIS_Z];
  const float init_theta = -asin( accel[AXIS_X] / sqrt( g2 ) );

  mtt_phi = init_phi;
  mtt_theta = init_theta;

  mtt_status = MT_STATUS_RUNNING;
}


void multitilt_predict( const float* gyro ) {
  /* unbias gyro */
  mtt_p = gyro[AXIS_P] - mtt_bp;
  mtt_q = gyro[AXIS_Q] - mtt_bq;
  mtt_r = gyro[AXIS_R] - mtt_br; 

  /* update angles */
  float s_phi = sin(mtt_phi);
  float c_phi = cos(mtt_phi);
  float t_theta = tan(mtt_theta);

  float phi_dot = mtt_p + s_phi*t_theta*mtt_q + c_phi*t_theta*mtt_r;
  mtt_phi += phi_dot * DT;
  float theta_dot = c_phi*mtt_q - s_phi*mtt_r;
  mtt_theta += theta_dot * DT;

  /* Pdot = A*P + P*A' + Q */
  const float P_phi_dot00 = Q_angle - mtt_P_phi[0][1] - mtt_P_phi[1][0];
  const float P_phi_dot01 = - mtt_P_phi[1][1];
  const float P_phi_dot10 = - mtt_P_phi[1][1];
  const float P_phi_dot11 = Q_bias;

  /* P += Pdot * dt */
  mtt_P_phi[0][0] += P_phi_dot00 * DT;
  mtt_P_phi[0][1] += P_phi_dot01 * DT;
  mtt_P_phi[1][0] += P_phi_dot10 * DT;
  mtt_P_phi[1][1] += P_phi_dot11 * DT;

  /* Pdot = A*P + P*A' + Q */
  const float P_theta_dot00 = Q_angle - mtt_P_theta[0][1] - mtt_P_theta[1][0];
  const float P_theta_dot01 = - mtt_P_theta[1][1];
  const float P_theta_dot10 = - mtt_P_theta[1][1];
  const float P_theta_dot11 = Q_bias;

  /* P += Pdot * dt */
  mtt_P_theta[0][0] += P_theta_dot00 * DT;
  mtt_P_theta[0][1] += P_theta_dot01 * DT;
  mtt_P_theta[1][0] += P_theta_dot10 * DT;
  mtt_P_theta[1][1] += P_theta_dot11 * DT;
}

void multitilt_update( const float* accel ) {

  const float measure_phi = atan2(accel[AXIS_Y], accel[AXIS_Z]);
  
  const float err_phi = measure_phi - mtt_phi;

  const float Pct_0_phi = mtt_P_phi[0][0];
  const float Pct_1_phi = mtt_P_phi[1][0];
                
  /* E = C P C' + R */
  const float E_phi = R_accel + Pct_0_phi;

  /* K = P C' inv(E) */
  const float K_0_phi = Pct_0_phi / E_phi;
  const float K_1_phi = Pct_1_phi / E_phi;

  /* P = P - K C P */
  const float t_0_phi = Pct_0_phi;
  const float t_1_phi = mtt_P_phi[0][1];

  mtt_P_phi[0][0] -= K_0_phi * t_0_phi;
  mtt_P_phi[0][1] -= K_0_phi * t_1_phi;
  mtt_P_phi[1][0] -= K_1_phi * t_0_phi;
  mtt_P_phi[1][1] -= K_1_phi * t_1_phi;
    
  /*  X += K * err */
  mtt_phi += K_0_phi * err_phi;
  mtt_bp  += K_1_phi * err_phi;
 



  const float g2 =					
    accel[AXIS_X]*accel[AXIS_X] +	
    accel[AXIS_Y]*accel[AXIS_Y] +	
    accel[AXIS_Z]*accel[AXIS_Z];
  const float measure_theta = -asin( accel[AXIS_X] / sqrt( g2 ) );

  const float err_theta = measure_theta - mtt_theta;

  const float Pct_0_theta = mtt_P_theta[0][0];
  const float Pct_1_theta = mtt_P_theta[1][0];

  /* E = C P C' + R */
  const float E_theta = R_accel + Pct_0_theta;

  /* K = P C' inv(E) */
  const float K_0_theta = Pct_0_theta / E_theta;
  const float K_1_theta = Pct_1_theta / E_theta;

  /* P = P - K C P */
  const float t_0_theta = Pct_0_theta;
  const float t_1_theta = mtt_P_theta[0][1];

  mtt_P_theta[0][0] -= K_0_theta * t_0_theta;
  mtt_P_theta[0][1] -= K_0_theta * t_1_theta;
  mtt_P_theta[1][0] -= K_1_theta * t_0_theta;
  mtt_P_theta[1][1] -= K_1_theta * t_1_theta;
    
  /*  X += K * err */
  mtt_theta += K_0_theta * err_theta;
  mtt_bq  += K_1_theta * err_theta;

 
}



