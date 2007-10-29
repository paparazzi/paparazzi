#include "booz_estimator.h"

#include "booz_inter_mcu.h"


float booz_estimator_uf_p;
float booz_estimator_uf_q;
float booz_estimator_uf_r;

float booz_estimator_p;
float booz_estimator_q;
float booz_estimator_r;

float booz_estimator_phi;
float booz_estimator_theta;
float booz_estimator_psi;

#ifndef DISABLE_NAV
float booz_estimator_dcm[AXIS_NB][AXIS_NB];

float booz_estimator_x;
float booz_estimator_y;
float booz_estimator_z;

float booz_estimator_vx;
float booz_estimator_vy;
float booz_estimator_vz;
#endif /* DISABLE_NAV */

void booz_estimator_init( void ) {

  booz_estimator_uf_p = 0.;
  booz_estimator_uf_q = 0.;
  booz_estimator_uf_r = 0.;

  booz_estimator_p = 0.;
  booz_estimator_q = 0.;
  booz_estimator_r = 0.;

  booz_estimator_phi = 0.;
  booz_estimator_theta = 0.;
  booz_estimator_psi = 0.;
#ifndef DISABLE_NAV
  booz_estimator_x = 0.;
  booz_estimator_y = 0.;
  booz_estimator_z = 0.;

  booz_estimator_vx = 0.;
  booz_estimator_vy = 0.;
  booz_estimator_vz = 0.;
#endif /* DISABLE_NAV */
}

void booz_estimator_read_inter_mcu_state( void ) {

  booz_estimator_uf_p  = inter_mcu_state.r_rates[AXIS_P] * M_PI/RATE_PI_S;
  booz_estimator_uf_q  = inter_mcu_state.r_rates[AXIS_Q] * M_PI/RATE_PI_S;
  booz_estimator_uf_r  = inter_mcu_state.r_rates[AXIS_R] * M_PI/RATE_PI_S;
  booz_estimator_p     = inter_mcu_state.f_rates[AXIS_P] * M_PI/RATE_PI_S;
  booz_estimator_q     = inter_mcu_state.f_rates[AXIS_Q] * M_PI/RATE_PI_S;
  booz_estimator_r     = inter_mcu_state.f_rates[AXIS_R] * M_PI/RATE_PI_S;
  booz_estimator_phi   = inter_mcu_state.f_eulers[AXIS_X] * M_PI/ANGLE_PI;
  booz_estimator_theta = inter_mcu_state.f_eulers[AXIS_Y] * M_PI/ANGLE_PI;
  booz_estimator_psi   = inter_mcu_state.f_eulers[AXIS_Z] * M_PI/ANGLE_PI;

}

void booz_estimator_compute_dcm( void ) {

  float sinPHI   = sin( booz_estimator_phi );
  float cosPHI   = cos( booz_estimator_phi );
  float sinTHETA = sin( booz_estimator_theta);
  float cosTHETA = cos( booz_estimator_theta);
  float sinPSI   = sin( booz_estimator_psi);
  float cosPSI   = cos( booz_estimator_psi);

  booz_estimator_dcm[0][0] = cosTHETA * cosPSI;
  booz_estimator_dcm[0][1] = cosTHETA * sinPSI;
  booz_estimator_dcm[0][2] = -sinTHETA;
  booz_estimator_dcm[1][0] = sinPHI * sinTHETA * cosPSI - cosPHI * sinPSI;
  booz_estimator_dcm[1][1] = sinPHI * sinTHETA * sinPSI + cosPHI * cosPSI;
  booz_estimator_dcm[1][2] = sinPHI * cosTHETA;
  booz_estimator_dcm[2][0] = cosPHI * sinTHETA * cosPSI + sinPHI * sinPSI;
  booz_estimator_dcm[2][1] = cosPHI * sinTHETA * sinPSI - sinPHI * cosPSI;
  booz_estimator_dcm[2][2] = cosPHI * cosTHETA;

}

#ifndef DISABLE_NAV
void booz_estimator_set_speed_and_pos(float _vx, float _vy, float _vz, float _x, float _y, float _z) {

  booz_estimator_vx = _vx;
  booz_estimator_vy = _vy;
  booz_estimator_vz = _vz;

  booz_estimator_x = _x;
  booz_estimator_y = _y;
  booz_estimator_z = _z;

}
#endif /* DISABLE_NAV */
