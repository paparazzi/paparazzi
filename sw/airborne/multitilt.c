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
float mtt_P_psi[2][2];

/* process covariance noise */
//static const float Q_angle = 0.0005;
static const float Q_angle = 0.00025;
static const float Q_bias  = 0.0015;
/* Measurement covariance */
//static const float R_accel = 0.3;
static const float R_accel = 0.4;

#define WRAP(x,a) { while (x > a) x -= 2 * a; while (x <= -a) x += 2 * a;}

static inline float phi_of_accel( const float* accel) {
  return atan2(accel[AXIS_Y], accel[AXIS_Z]);
}

static inline float theta_of_accel( const float* accel) {
  const float g2 =					
    accel[AXIS_X]*accel[AXIS_X] +	
    accel[AXIS_Y]*accel[AXIS_Y] +	
    accel[AXIS_Z]*accel[AXIS_Z];
  return -asin( accel[AXIS_X] / sqrt( g2 ) );
}

static inline float psi_of_mag( const int16_t* mag) {			
    /* untilt magnetometer */			
    const float ctheta  = cos(  mtt_theta );	
    const float stheta  = sin( mtt_theta );	
    const float cphi  = cos( mtt_phi );		
    const float sphi  = sin( mtt_phi );		
						
    const float mn =				
      ctheta*      mag[0]+			
      sphi*stheta* mag[1]+			
      cphi*stheta* mag[2];			
    const float me =				
      /*    0*     mag[0]+ */			
      cphi*  mag[1]+				
      -sphi* mag[2];				
    return -atan2( me, mn );			
}

void multitilt_init(void) {
  mtt_status = MT_STATUS_UNINIT;
  mtt_phi = 0.;
  mtt_theta = 0.;
  mtt_psi = 0.;

  mtt_p = 0.;
  mtt_q = 0.;
  mtt_r = 0.;

  mtt_bp = 0.;
  mtt_bq = 0.;
  mtt_br = 0.;
}

void multitilt_start(const float* accel, const float* gyro, const int16_t* mag) {
  /* reset covariance matrices */
  const float cov_init[2][2] = {{1., 0.},
				{0., 1.}};
  memcpy(mtt_P_phi, cov_init, sizeof(cov_init));
  memcpy(mtt_P_theta, cov_init, sizeof(cov_init));
  memcpy(mtt_P_psi, cov_init, sizeof(cov_init));

  /* initialise state */
  mtt_p = 0.;
  mtt_q = 0.;
  mtt_r = 0.;
  
  mtt_bp = gyro[AXIS_P];
  mtt_bq = gyro[AXIS_Q];
  mtt_br = gyro[AXIS_R];

  const float init_phi = phi_of_accel(accel);
  const float init_theta = theta_of_accel(accel);
#ifndef DISABLE_MAGNETOMETER
  const float init_psi = psi_of_mag(mag);
#endif
  mtt_phi = init_phi;
  mtt_theta = init_theta;
#ifndef DISABLE_MAGNETOMETER
  mtt_psi = init_psi;
#endif

  mtt_status = MT_STATUS_RUNNING;
}


static inline void mtt_predict_axis(float* angle, float angle_dot, float P[2][2]) {

  (*angle) += angle_dot * DT;

  const float P_dot00 = Q_angle - P[0][1] - P[1][0];
  const float P_dot01 = - P[1][1];
  const float P_dot10 = - P[1][1];
  const float P_dot11 = Q_bias;

  /* P += Pdot * dt */
  P[0][0] += P_dot00 * DT;
  P[0][1] += P_dot01 * DT;
  P[1][0] += P_dot10 * DT;
  P[1][1] += P_dot11 * DT;

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
  float theta_dot = c_phi*mtt_q - s_phi*mtt_r;
  mtt_predict_axis(&mtt_phi, phi_dot, mtt_P_phi);
  mtt_predict_axis(&mtt_theta, theta_dot, mtt_P_theta);

#ifndef DISABLE_MAGNETOMETER
  float c_theta = cos(mtt_theta);
  float psi_dot = s_phi/c_theta*mtt_q + c_phi/c_theta*mtt_r;
  mtt_predict_axis(&mtt_psi, psi_dot, mtt_P_psi);
#endif


}

static void inline MttUpdateAxis(float _err, float _P[2][2], float* angle, float* bias) {
  const float Pct_0 = _P[0][0];
  const float Pct_1 = _P[1][0];
  /* E = C P C' + R */
  const float E = R_accel + Pct_0;
  /* K = P C' inv(E) */
  const float K_0 = Pct_0 / E;
  const float K_1 = Pct_1 / E;
  /* P = P - K C P */
  const float t_0 = Pct_0;
  const float t_1 = _P[0][1];

  _P[0][0] -= K_0 * t_0;
  _P[0][1] -= K_0 * t_1;
  _P[1][0] -= K_1 * t_0;
  _P[1][1] -= K_1 * t_1;
    
  /*  X += K * err */
  (*angle) += K_0 * _err;
  (*bias)  += K_1 * _err;

}

void multitilt_update( const float* accel, const int16_t* mag ) {

  const float measure_phi = phi_of_accel(accel);
  float err_phi = measure_phi - mtt_phi;
  WRAP(err_phi, M_PI);
  MttUpdateAxis(err_phi, mtt_P_phi, &mtt_phi, &mtt_bp);
  WRAP(mtt_phi, M_PI);

  const float measure_theta = theta_of_accel(accel);
  float err_theta = measure_theta - mtt_theta;
  WRAP(err_theta, M_PI_2);
  MttUpdateAxis(err_theta, mtt_P_theta, &mtt_theta, &mtt_bq);
  WRAP(mtt_theta, M_PI_2);

#ifndef DISABLE_MAGNETOMETER
  float measure_psi = psi_of_mag(mag);
  float err_psi = measure_psi - mtt_psi;
  WRAP(err_psi, M_PI);
  MttUpdateAxis(err_psi, mtt_P_psi, &mtt_psi, &mtt_br);
  WRAP(mtt_psi, M_PI);
#endif
}



