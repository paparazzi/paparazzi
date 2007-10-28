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
