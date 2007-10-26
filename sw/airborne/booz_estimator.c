#include "booz_estimator.h"

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
