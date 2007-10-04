#include "multitilt.h"

#include <string.h>
#include <math.h>
#include "imu_v3.h"

uint8_t mt_status;

/* FIXME */
#define DT 0.01

/* attitude */
float mt_phi;
float mt_theta;
float mt_psi;
/* unbiased rates */
float mt_p;
float mt_q;
float mt_r;
/* gyro biases */
float mt_bp;
float mt_bq;
float mt_br;
/* covariance matrix */
float mt_P_phi[2][2];
float mt_P_theta[2][2];
float mt_P_psi[2][2];
/* process covariance noise */
static const float Q_angle = 0.001;
static const float Q_gyro  = 0.003;
/* Measurement covariance */
static const float R_accel = 0.3;


/* initialisation stuff */
#define MT_INIT_NB_SAMPLES 32
#define MT_INIT_MAX_VAR_PHI 0.01
static float mt_phi_init_samples[MT_INIT_NB_SAMPLES];
static float mt_phi_init_sum;
static float mt_p_init_samples[MT_INIT_NB_SAMPLES];
static float mt_p_init_sum;
static uint8_t mt_init_head;
static bool_t mt_init_buf_filled;

static void multitilt_initialise(void);
static void multitilt_propagate(void);
static void multitilt_update(void);

void multitilt_reset(void) {
  const float cov_init[2][2];
  memcpy(mt_P_phi, cov_init, sizeof(cov_init));
  memcpy(mt_P_theta, cov_init, sizeof(cov_init));
  memcpy(mt_P_psi, cov_init, sizeof(cov_init));

  mt_status = MT_STATUS_UNINIT;
  mt_init_buf_filled = FALSE;
  mt_init_head = 0;
  mt_phi_init_sum = 0.;
  mt_p_init_sum = 0.;
}

void multitilt_run(void) {
  

}

static void multitilt_propagate(void) {
  /* unbias gyro */
  mt_p = imu_gyro[AXIS_P] - mt_bp;

  /* update angle */
  mt_phi += mt_p * DT;

  /* Pdot = A*P + P*A' + Q */
  const float Pdot00 = Q_angle - mt_P_phi[0][1] - mt_P_phi[1][0];
  const float Pdot01 = - mt_P_phi[1][1];
  const float Pdot10 = - mt_P_phi[1][1];
  const float Pdot11 = Q_gyro;

  /* P += Pdot * dt */
  mt_P_phi[0][0] += Pdot00 * DT;
  mt_P_phi[0][1] += Pdot01 * DT;
  mt_P_phi[1][0] += Pdot10 * DT;
  mt_P_phi[1][1] += Pdot11 * DT;
}

static void multitilt_update(void) {

  const float measure_phi = atan2(imu_accel[AXIS_Y], imu_accel[AXIS_Z]);
  
  const float err_phi = measure_phi - mt_phi;

  const float C_0 = 1.;

  const float PCt_0 = C_0 * mt_P_phi[0][0];
  const float PCt_1 = C_0 * mt_P_phi[1][0];
                
  /* E = C P C' + R */
  const float E = R_accel + C_0 * PCt_0;

  /* K = P C' inv(E) */
  const float K_0 = PCt_0 / E;
  const float K_1 = PCt_1 / E;

  /* P = P - K C P */
  const float t_0 = PCt_0;
  const float t_1 = C_0 *  mt_P_phi[0][1];

  mt_P_phi[0][0] -= K_0 * t_0;
  mt_P_phi[0][1] -= K_0 * t_1;
  mt_P_phi[1][0] -= K_1 * t_0;
  mt_P_phi[1][1] -= K_1 * t_1;
    
  /*  X += K * err */
  mt_phi += K_0 * err_phi;
  mt_bq  += K_1 * err_phi;
  
}



static void multitilt_initialise(void) {

  mt_init_head++;
  if (mt_init_head >= MT_INIT_NB_SAMPLES) {
    mt_init_buf_filled = TRUE;
    mt_init_head = 0;
  }

  const float m_phi = atan2(imu_accel[AXIS_Y], imu_accel[AXIS_Z]);
  mt_phi_init_sum -= mt_phi_init_samples[mt_init_head];
  mt_phi_init_samples[mt_init_head] =  m_phi;
  mt_phi_init_sum += m_phi;
  
  mt_p_init_sum -= mt_p_init_samples[mt_init_head];
  mt_p_init_samples[mt_init_head] = imu_gyro[AXIS_P];
  mt_p_init_sum += imu_gyro[AXIS_P];
  
  if (mt_init_buf_filled) {
    float avg_phi = mt_phi_init_sum / MT_INIT_NB_SAMPLES;
    float var_phi = 0.;
    uint8_t i;
    for (i=0; i < MT_INIT_NB_SAMPLES; i++) {
      float diff = mt_phi_init_samples[i] - avg_phi;
      var_phi += (diff * diff);
    }
    
    if (var_phi < MT_INIT_MAX_VAR_PHI) {
      mt_phi = avg_phi;
      mt_bp = mt_p_init_sum / MT_INIT_NB_SAMPLES;
      mt_status = MT_STATUS_RUNNING;
    }
  }
}
