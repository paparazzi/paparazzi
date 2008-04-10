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

float booz_estimator_dcm[AXIS_NB][AXIS_NB];

float booz_estimator_x;
float booz_estimator_y;
float booz_estimator_z;

float booz_estimator_vx;
float booz_estimator_vy;
float booz_estimator_vz;

float booz_estimator_u;
float booz_estimator_v;
float booz_estimator_w;

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

  booz_estimator_x = 0.;
  booz_estimator_y = 0.;
  booz_estimator_z = 0.;

  booz_estimator_vx = 0.;
  booz_estimator_vy = 0.;
  booz_estimator_vz = 0.;

  booz_estimator_u = 0.;
  booz_estimator_v = 0.;
  booz_estimator_w = 0.;
}


#define BoozEstimatorComputeDCM() {			                     \
							                     \
    float sinPHI   = sin( booz_estimator_phi );		                     \
    float cosPHI   = cos( booz_estimator_phi );		                     \
    float sinTHETA = sin( booz_estimator_theta);	                     \
    float cosTHETA = cos( booz_estimator_theta);	                     \
    float sinPSI   = sin( booz_estimator_psi);		                     \
    float cosPSI   = cos( booz_estimator_psi);		                     \
    							                     \
    booz_estimator_dcm[0][0] = cosTHETA * cosPSI;			     \
    booz_estimator_dcm[0][1] = cosTHETA * sinPSI;			     \
    booz_estimator_dcm[0][2] = -sinTHETA;				     \
    booz_estimator_dcm[1][0] = sinPHI * sinTHETA * cosPSI - cosPHI * sinPSI; \
    booz_estimator_dcm[1][1] = sinPHI * sinTHETA * sinPSI + cosPHI * cosPSI; \
    booz_estimator_dcm[1][2] = sinPHI * cosTHETA;			     \
    booz_estimator_dcm[2][0] = cosPHI * sinTHETA * cosPSI + sinPHI * sinPSI; \
    booz_estimator_dcm[2][1] = cosPHI * sinTHETA * sinPSI - sinPHI * cosPSI; \
    booz_estimator_dcm[2][2] = cosPHI * cosTHETA;			     \
									     \
}

#define BoozEstimatorUpdateBodySpeed() {				\
    						                        \
    booz_estimator_u =							\
      booz_estimator_dcm[AXIS_U][AXIS_X] * booz_estimator_vx +		\
      booz_estimator_dcm[AXIS_U][AXIS_Y] * booz_estimator_vy +		\
      booz_estimator_dcm[AXIS_U][AXIS_Z] * booz_estimator_vz ;		\
    									\
    booz_estimator_v =							\
      booz_estimator_dcm[AXIS_V][AXIS_X] * booz_estimator_vx +		\
      booz_estimator_dcm[AXIS_V][AXIS_Y] * booz_estimator_vy +		\
      booz_estimator_dcm[AXIS_V][AXIS_Z] * booz_estimator_vz ;		\
    									\
    booz_estimator_w =							\
      booz_estimator_dcm[AXIS_W][AXIS_X] * booz_estimator_vx +		\
      booz_estimator_dcm[AXIS_W][AXIS_Y] * booz_estimator_vy +		\
      booz_estimator_dcm[AXIS_W][AXIS_Z] * booz_estimator_vz ;		\
    									\
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

  booz_estimator_x = inter_mcu_state.pos[AXIS_X]; 
  booz_estimator_y = inter_mcu_state.pos[AXIS_Y]; 
  booz_estimator_z = inter_mcu_state.pos[AXIS_Z]; 

  booz_estimator_vx = inter_mcu_state.speed[AXIS_X]; 
  booz_estimator_vy = inter_mcu_state.speed[AXIS_Y]; 
  booz_estimator_vz = inter_mcu_state.speed[AXIS_Z]; 

  BoozEstimatorComputeDCM();

  BoozEstimatorUpdateBodySpeed();
  
}


