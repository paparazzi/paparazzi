#include "tl_estimator.h"
#include "gps.h"
#include "tl_imu.h"
#include "flight_plan.h"
#include "tl_control.h"
#include "tl_vfilter.h"

#include "tl_telemetry.h"
#include "led.h"

bool_t estimator_in_flight;
bool_t tl_estimator_set_ground_pressure;
uint16_t estimator_flight_time;

float tl_estimator_u;
float tl_estimator_v;
float tl_estimator_u_dot;
float tl_estimator_v_dot;

float estimator_x; /* m */
float estimator_y; /* m */
float estimator_z; /* altitude in m */
float tl_estimator_agl; /* AGL in m */
float tl_estimator_agl_dot; /* AGL in m/s */

float estimator_speed; /* m/s */
float estimator_climb; /* m/s */
float estimator_course; /* rad, CCW */

float estimator_speed_east;
float estimator_speed_north;

float estimator_r;
float estimator_psi; /* rad, CCW */ 
float estimator_z_baro;
float estimator_ground_pressure;

static float estimator_cos_psi;
static float estimator_sin_psi;

#define TL_ESTIMATOR_CRUISE_POWER (0.73*MAX_PPRZ)
float tl_estimator_cruise_power;

#define DT_UPDATE (1./60.)
static const float Q_angle = 0.00025;
static const float Q_bias  = 0.0005;
static const float R_accel = 0.5;

#define TL_PSI_KALM_UNINIT  0
#define TL_PSI_KALM_RUNNING 1

#define BARO_ALPHA 1

float tl_baro_alpha;

uint8_t tl_psi_kalm_status;
float   tl_psi_kalm_psi;
float   tl_psi_kalm_r;
float   tl_psi_kalm_bias;
float   tl_psi_kalm_P[2][2];


static inline void compute_dcm(void) {
  estimator_cos_psi = cos(estimator_psi);
  estimator_sin_psi = sin(estimator_psi);
}

void tl_estimator_to_body_frame(float east, float north,
				float *front, float *right) {
  *front = estimator_cos_psi * north + estimator_sin_psi * east;
  *right = - estimator_sin_psi * north + estimator_cos_psi * east;
}

void tl_estimator_periodic_task(void) {
  estimator_in_flight = tl_control_power_sp > 500;
}


void tl_estimator_init(void) {
  estimator_psi = 0.;
  compute_dcm();
  tl_estimator_u = 0.;
  tl_estimator_v = 0.;
  tl_estimator_u_dot = 0.;
  tl_estimator_v_dot = 0.;
  tl_estimator_cruise_power = TL_ESTIMATOR_CRUISE_POWER;
  tl_estimator_set_ground_pressure = TRUE;
  tl_baro_alpha = BARO_ALPHA;
}

void tl_estimator_use_gps(void) {
  float gps_east = gps_utm_east / 100.;
  float gps_north = gps_utm_north / 100.;

  /* Relative position to reference */
  estimator_x = gps_east - NAV_UTM_EAST0;
  estimator_y = gps_north - NAV_UTM_NORTH0;
  estimator_z = gps_alt / 100.;

  estimator_speed = gps_gspeed / 100.;
  estimator_climb = gps_climb / 100.;
  estimator_course = RadOfDeg(gps_course / 10.);

  estimator_speed_east = estimator_speed * sin(estimator_course);
  estimator_speed_north = estimator_speed * cos(estimator_course);

  tl_estimator_to_body_frame(estimator_speed_east, estimator_speed_north, &tl_estimator_u, &tl_estimator_v);
}

void tl_estimator_use_gyro(void) {
  estimator_r = tl_imu_r;
  if (tl_psi_kalm_status == TL_PSI_KALM_RUNNING) {
    tl_psi_kalm_propagate(tl_imu_r);
    estimator_psi = tl_psi_kalm_psi;
    estimator_r = tl_psi_kalm_r;
    compute_dcm();
  }
  tl_vf_predict(tl_imu_accel);

  DOWNLINK_SEND_VF_PREDICT(&tl_imu_accel);

  tl_estimator_agl = tl_vf_z;
  tl_estimator_agl_dot = tl_vf_zdot;
}

void tl_estimator_use_imu(void) {
  float estimator_psi_measure = -atan2(tl_imu_hx, -tl_imu_hy) + MAGNETIC_DECLINATION;
  
  if (tl_estimator_set_ground_pressure) {
    estimator_ground_pressure = tl_imu_pressure;
    estimator_z_baro = 0;
  }

  estimator_z_baro = (estimator_ground_pressure - (float)tl_imu_pressure)*0.084;

  if (tl_psi_kalm_status == TL_PSI_KALM_UNINIT) {
    tl_psi_kalm_start(estimator_r, estimator_psi);
    estimator_psi = estimator_psi_measure;
    compute_dcm();
  } else {
    tl_psi_kalm_update(estimator_psi_measure);
  }
#if 0
  tl_vf_update(-tl_imu_rm);
#else
  tl_vf_update(-estimator_z_baro);
  DOWNLINK_SEND_VF_UPDATE(&estimator_z_baro, &tl_imu_rm);
#endif
}




void tl_psi_kalm_init(void) {
  tl_psi_kalm_status = TL_PSI_KALM_UNINIT;
}

void tl_psi_kalm_start( float gyro, float angle) {
  tl_psi_kalm_psi = angle;
  tl_psi_kalm_bias = gyro;
  tl_psi_kalm_r = 0.;
  tl_psi_kalm_P[0][0] = 1.;
  tl_psi_kalm_P[0][1] = 0.;
  tl_psi_kalm_P[1][0] = 0.;
  tl_psi_kalm_P[0][1] = 1.;
  tl_psi_kalm_status = TL_PSI_KALM_RUNNING;

}

void tl_psi_kalm_propagate( float gyro) {
  /* r_est = r_measure - bias */
  tl_psi_kalm_r = gyro - tl_psi_kalm_bias;

  /* psi += r * dt */
  tl_psi_kalm_psi += (tl_psi_kalm_r*DT_UPDATE);
  NormRadAngle(tl_psi_kalm_psi);

  const float P_dot00 = Q_angle - tl_psi_kalm_P[0][1] - tl_psi_kalm_P[1][0];
  const float P_dot01 = - tl_psi_kalm_P[1][1];
  const float P_dot10 = - tl_psi_kalm_P[1][1];
  const float P_dot11 = Q_bias;
  /* P += Pdot * dt */
  tl_psi_kalm_P[0][0] += (P_dot00 * DT_UPDATE);
  tl_psi_kalm_P[0][1] += (P_dot01 * DT_UPDATE);
  tl_psi_kalm_P[1][0] += (P_dot10 * DT_UPDATE);
  tl_psi_kalm_P[1][1] += (P_dot11 * DT_UPDATE);
}

//#define PROPAGATE_ONLY 1

void tl_psi_kalm_update( float angle) {
#ifndef PROPAGATE_ONLY
  float err = angle - tl_psi_kalm_psi;
  NormRadAngle(err);
  const float Pct_0 = tl_psi_kalm_P[0][0];
  const float Pct_1 = tl_psi_kalm_P[1][0];
  /* E = C P C' + R */
  const float E = R_accel + Pct_0;
  /* K = P C' inv(E) */
  const float K_0 = Pct_0 / E;
  const float K_1 = Pct_1 / E;
  /* P = P - K C P */
  const float t_0 = Pct_0;
  const float t_1 = tl_psi_kalm_P[0][1];

  tl_psi_kalm_P[0][0] -= K_0 * t_0;
  tl_psi_kalm_P[0][1] -= K_0 * t_1;
  tl_psi_kalm_P[1][0] -= K_1 * t_0;
  tl_psi_kalm_P[1][1] -= K_1 * t_1;
    
  /*  X += K * err */
  tl_psi_kalm_psi += K_0 * err;
  tl_psi_kalm_bias  += K_1 * err;
#endif
}

