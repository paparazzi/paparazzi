#include "booz_ahrs.h"

#include <string.h>
#include <math.h>

#include "6dof.h"

#define DT 4e-3

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

static inline void multitilt_update( const float* accel, const int16_t* mag );
static inline void mtt_update_axis(float _err, float _P[2][2], float* angle, float* bias);
static inline void multitilt_predict( const float* gyro );
static inline void mtt_predict_axis(float* angle, float angle_dot, float P[2][2]);


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
  const float ctheta  = cos(  booz_ahrs_theta );	
  const float stheta  = sin( booz_ahrs_theta );	
  const float cphi  = cos( booz_ahrs_phi );		
  const float sphi  = sin( booz_ahrs_phi );		
						
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

void booz_ahrs_init(void) {
  booz_ahrs_status = BOOZ_AHRS_STATUS_UNINIT;
  booz_ahrs_phi = 0.;
  booz_ahrs_theta = 0.;
  booz_ahrs_psi = 0.;

  booz_ahrs_p = 0.;
  booz_ahrs_q = 0.;
  booz_ahrs_r = 0.;

  booz_ahrs_bp = 0.;
  booz_ahrs_bq = 0.;
  booz_ahrs_br = 0.;
}

void booz_ahrs_start(const float* accel, const float* gyro, const int16_t* mag) {
  /* reset covariance matrices */
  const float cov_init[2][2] = {{1., 0.},
				{0., 1.}};
  memcpy(mtt_P_phi, cov_init, sizeof(cov_init));
  memcpy(mtt_P_theta, cov_init, sizeof(cov_init));
  memcpy(mtt_P_psi, cov_init, sizeof(cov_init));

  /* initialise state */
  booz_ahrs_p = 0.;
  booz_ahrs_q = 0.;
  booz_ahrs_r = 0.;
  
  booz_ahrs_bp = gyro[AXIS_P];
  booz_ahrs_bq = gyro[AXIS_Q];
  booz_ahrs_br = gyro[AXIS_R];

  const float init_phi = phi_of_accel(accel);
  const float init_theta = theta_of_accel(accel);
#ifndef DISABLE_MAGNETOMETER
  const float init_psi = psi_of_mag(mag);
#endif
  booz_ahrs_phi = init_phi;
  booz_ahrs_theta = init_theta;
#ifndef DISABLE_MAGNETOMETER
  booz_ahrs_psi = init_psi;
#endif

  booz_ahrs_status = BOOZ_AHRS_STATUS_RUNNING;
}

void booz_ahrs_run(const float* accel, const float* gyro, const int16_t* mag) {
  multitilt_predict(gyro);
  multitilt_update(accel, mag);
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


static inline void multitilt_predict( const float* gyro ) {
  /* unbias gyro */
  booz_ahrs_p = gyro[AXIS_P] - booz_ahrs_bp;
  booz_ahrs_q = gyro[AXIS_Q] - booz_ahrs_bq;
  booz_ahrs_r = gyro[AXIS_R] - booz_ahrs_br; 

  /* update angles */
  float s_phi = sin(booz_ahrs_phi);
  float c_phi = cos(booz_ahrs_phi);
  float t_theta = tan(booz_ahrs_theta);

  float phi_dot = booz_ahrs_p + s_phi*t_theta*booz_ahrs_q + c_phi*t_theta*booz_ahrs_r;
  float theta_dot = c_phi*booz_ahrs_q - s_phi*booz_ahrs_r;
  mtt_predict_axis(&booz_ahrs_phi, phi_dot, mtt_P_phi);
  mtt_predict_axis(&booz_ahrs_theta, theta_dot, mtt_P_theta);

#ifndef DISABLE_MAGNETOMETER
  float c_theta = cos(booz_ahrs_theta);
  float psi_dot = s_phi/c_theta*booz_ahrs_q + c_phi/c_theta*booz_ahrs_r;
  mtt_predict_axis(&booz_ahrs_psi, psi_dot, mtt_P_psi);
#endif


}

static inline void mtt_update_axis(float _err, float _P[2][2], float* angle, float* bias) {
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

static inline void multitilt_update( const float* accel, const int16_t* mag ) {

  booz_ahrs_measure_phi = phi_of_accel(accel);
  float err_phi = booz_ahrs_measure_phi - booz_ahrs_phi;
  WRAP(err_phi, M_PI);
  mtt_update_axis(err_phi, mtt_P_phi, &booz_ahrs_phi, &booz_ahrs_bp);
  WRAP(booz_ahrs_phi, M_PI);

  booz_ahrs_measure_theta = theta_of_accel(accel);
  float err_theta = booz_ahrs_measure_theta - booz_ahrs_theta;
  WRAP(err_theta, M_PI_2);
  mtt_update_axis(err_theta, mtt_P_theta, &booz_ahrs_theta, &booz_ahrs_bq);
  WRAP(booz_ahrs_theta, M_PI_2);

#ifndef DISABLE_MAGNETOMETER
  booz_ahrs_measure_psi = psi_of_mag(mag);
  float err_psi = booz_ahrs_measure_psi - booz_ahrs_psi;
  WRAP(err_psi, M_PI);
  mtt_update_axis(err_psi, mtt_P_psi, &booz_ahrs_psi, &booz_ahrs_br);
  WRAP(booz_ahrs_psi, M_PI);
#endif

}



